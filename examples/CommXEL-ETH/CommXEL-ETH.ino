/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <XELNetworkMaster.h>
#include <EEPROM.h>

#define DEBUG_SERIAL     Serial1
#define DEBUG_BEGIN(x)   DEBUG_SERIAL.begin(x)
#define DEBUG_PRINT(x)   DEBUG_SERIAL.print(x)
#define DEBUG_PRINTLN(x) DEBUG_SERIAL.println(x)
extern bool enable_auto_reset;

// Communication port setting for communication with ROS2.
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <EthernetUdp.h>

bool dhcp_enable = true;
const uint8_t default_mac[6] = {0x00, 0x08, 0xDC, 0x50, 0xF4, 0xF5};
const uint8_t* p_mac = default_mac;
IPAddress static_ip;
const char* p_static_ip = "192.168.0.4";

// Definition of information for connecting to MICRO-XRCE-DDS.
const char* p_ros2_node_name = "xelnetwork";
const char* p_agent_ip = "AGENT_IP_ADDRESS";
uint16_t agent_port = 2019; //AGENT port number

// Settings to communicate with other SLAVE devices (SensorXEL, etc.)
// by acting as master of the DYNAMIXEL protocol.
uint32_t dxl_master_baudrate = 1000000;
float dxl_master_protocol_ver = 2.0;

// Interval time setting to scan slave devices periodically for Plug And Play.
// If you set the parameter to 0, scans only at begin time and no longer scans.
uint32_t auto_scan_interval_ms = 200;
// The Scan function pings only one ID at a set interval.
// Therefore, by setting the range of IDs to scan,
// you can scan the same ID in shorter periods.
uint8_t start_id_to_scan = 0;
uint8_t end_id_to_scan = 5;

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udp;
static char config_assigned_ip[16];

uint32_t pre_time_led, pre_time;
enum OP_MODE{
  OP_XELNETWORK = 0,
  OP_BYPASS_USB_DXL
};
uint8_t op_mode = OP_XELNETWORK;
uint8_t pre_op_mode;

void setup() {
  // put your setup code here, to run once:
  pre_op_mode = op_mode;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);
  DEBUG_BEGIN(115200);

  getConfigFromEEPROM();
  beginDXLSlaveToConfig();

  while(XELNetworkMaster::initDXLMaster(DEFAULT_DXL_SERIAL, DEFAULT_DXL_PIN) == false)
  {
    printErrAndLED(" [Fail] XELNetworkMaster::initDXLMaster", 50);
    runDXLSlaveToConfig();
  }
  DEBUG_PRINTLN("[Success] XELNetworkMaster::initDXLMaster");

  checkButtonAndChangeMode();

  // start the Ethernet connection:
  bool ethernet_init = false;
  if(dhcp_enable){
    ethernet_init = Ethernet.begin((uint8_t*)p_mac);
  }else{
    Ethernet.begin((uint8_t*)p_mac, static_ip);
    ethernet_init = true;
  }
  
  if(ethernet_init){
    DEBUG_PRINT("[Success] Ethernet begin: ");
    DEBUG_PRINT(Ethernet.localIP().operator[](0)); DEBUG_PRINT('.');
    DEBUG_PRINT(Ethernet.localIP().operator[](1)); DEBUG_PRINT('.');
    DEBUG_PRINT(Ethernet.localIP().operator[](2)); DEBUG_PRINT('.');
    DEBUG_PRINT(Ethernet.localIP().operator[](3));
    DEBUG_PRINTLN();
    sprintf(config_assigned_ip, "%d.%d.%d.%d", 
      Ethernet.localIP().operator[](0),
      Ethernet.localIP().operator[](1),
      Ethernet.localIP().operator[](2),
      Ethernet.localIP().operator[](3));
  }else{
    sprintf(config_assigned_ip, "000.000.000.000");
  }

  while(ethernet_init == false){
    checkButtonAndChangeMode();
    if(op_mode == OP_XELNETWORK){
      // To change the settings even if the connection with the AP is not possible.
      runDXLSlaveToConfig();
      // LED
      if(millis() - pre_time_led >= 500){
        pre_time_led = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }else if(op_mode == OP_BYPASS_USB_DXL){
      serial2DXL();
      // LED
      if(millis() - pre_time_led >= 200){
        pre_time_led = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }
  }

  while(op_mode == OP_BYPASS_USB_DXL){
    checkButtonAndChangeMode();
    serial2DXL();
    // LED
    if(millis() - pre_time_led >= 200){
      pre_time_led = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }    
  }

  while(XELNetworkMaster::initROS2(udp, p_agent_ip, agent_port) == false)
  {
    printErrAndLED(" [Fail] XELNetworkMaster::initROS2", 50);
    runDXLSlaveToConfig();
  }
  DEBUG_PRINTLN("[Success] XELNetworkMaster::initROS2");

  while(XELNetworkMaster::initNode(p_ros2_node_name) == false)
  {
    DEBUG_PRINTLN(" [Fail] XELNetworkMaster::initNode");
    pre_time = millis();
    while(millis()-pre_time < 10000){
      // LED
      if(millis() - pre_time_led >= 50){
        pre_time_led = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }      
      runDXLSlaveToConfig();
    }
    DEBUG_PRINTLN("\t [INFO] Retry initNode");
  }
  DEBUG_PRINTLN("[Success] XELNetworkMaster::initNode");
  DEBUG_SERIAL.flush();

  while(XELNetworkMaster::initScan(start_id_to_scan, end_id_to_scan, auto_scan_interval_ms) == false)
  {
    printErrAndLED(" [Fail] XELNetworkMaster::initScan", 50);
    runDXLSlaveToConfig();
  }
  DEBUG_PRINTLN("[Success] XELNetworkMaster::initScan");

  while(XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver) == false)
  {
    printErrAndLED(" [Fail] XELNetworkMaster::begin", 50);
    runDXLSlaveToConfig();
  }
  DEBUG_PRINTLN("[Success] XELNetworkMaster::begin");
}

void loop() {
  // put your main code here, to run repeatedly:
  checkButtonAndChangeMode();
  if(op_mode == OP_XELNETWORK){  
    // Communication for master configuration(network, scan method, etc).
    runDXLSlaveToConfig();
    // Detect each XEL and act as a bridge between XEL and ROS2.
    XELNetworkMaster::run();
    // LED
    if(millis() - pre_time_led >= 1000){
      pre_time_led = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }else if(op_mode == OP_BYPASS_USB_DXL){
    serial2DXL();
    // LED
    if(millis() - pre_time_led >= 200){
      pre_time_led = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}



// Slave and EEPROM setting to allow to set/get WiFi and Agent IP settings 
// through the DYNAMIXEL protocol.
#include <EEPROM.h>
USBSerial& DXL_SLAVE_SERIAL = Serial;
const uint16_t COMMXEL_ETH_MODEL_NUM = 0x5043;
DYNAMIXEL::USBSerialPortHandler dxl_slave_port(DXL_SLAVE_SERIAL);
DYNAMIXEL::Slave dxl_slave(COMMXEL_ETH_MODEL_NUM);

enum CommXELItemAddr{
  // ADDR_ITEM_MODEL_NUMBER    = 0, //Default setting in the Slave class.
  // ADDR_ITEM_FIRMWARE_VER    = 6, //Default setting in the Slave class.
  // ADDR_ITEM_ID              = 7, //Default setting in the Slave class.
  ADDR_DXL_SLAVE_BAUDRATE        = 8,   //1byte
  ADDR_ROS2_NODE_NAME            = 10,  //32byte char
  ADDR_AUTO_SCAN_START_ID        = 50,  //uint8_t
  ADDR_AUTO_SCAN_END_ID          = 51,  //uint8_t
  ADDR_AUTO_SCAN_INTERVAL_MS     = 52,  //uint32_t
  ADDR_DXL_MASTER_BAUDRATE       = 58,  //uint32_t
  ADDR_DXL_MASTER_PROTOCOL       = 59,  //uint8_t
  ADDR_ITEM_DHCP_ENABLE          = 60,  //1byte
  ADDR_ITEM_MAC_ADDR             = 61,  //6bytes
  ADDR_ITEM_STATIC_IP            = 92,  //16byte char "xxx.xxx.xxx.xxx"
  ADDR_ITEM_ASSIGNED_IP          = 108, //16byte char "xxx.xxx.xxx.xxx"
  ADDR_MICRO_XRCE_DDS_AGENT_IP   = 124, //16byte char "xxx.xxx.xxx.xxx"
  ADDR_MICRO_XRCE_DDS_AGENT_PORT = 140  //uint16_t
};

// static uint8_t config_dxl_slave_baudrate_idx;
static char config_ros2_node_name[32];
static uint8_t config_start_id_to_scan;
static uint8_t config_end_id_to_scan;
static uint32_t config_auto_scan_interval_ms;
static uint8_t config_dxl_master_baudrate_idx;
static uint8_t config_dxl_master_protocol_ver;
static char config_mac_addr[6];
static uint8_t config_dhcp_enable;
static char config_static_ip[16];
static char config_u_xrce_agent_ip[16];
static uint16_t config_u_xrce_agent_port;


void beginDXLSlaveToConfig()
{
  dxl_slave_port.begin();
  dxl_slave.setPort((DXLPortHandler*)&dxl_slave_port);

  dxl_slave.setFirmwareVersion(1);
  dxl_slave.setID(200);

  // dxl_slave.addControlItem(ADDR_DXL_SLAVE_BAUDRATE, (uint8_t*)&config_dxl_slave_baudrate_idx, sizeof(config_dxl_slave_baudrate_idx));
  dxl_slave.addControlItem(ADDR_ROS2_NODE_NAME, (uint8_t*)config_ros2_node_name, sizeof(config_ros2_node_name));
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_START_ID, config_start_id_to_scan);
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_END_ID, config_end_id_to_scan);
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_INTERVAL_MS, config_auto_scan_interval_ms);
  dxl_slave.addControlItem(ADDR_DXL_MASTER_BAUDRATE, config_dxl_master_baudrate_idx);
  dxl_slave.addControlItem(ADDR_DXL_MASTER_PROTOCOL, config_dxl_master_protocol_ver);
  dxl_slave.addControlItem(ADDR_ITEM_DHCP_ENABLE, config_dhcp_enable);
  dxl_slave.addControlItem(ADDR_ITEM_MAC_ADDR, (uint8_t*)config_mac_addr, sizeof(config_mac_addr));
  dxl_slave.addControlItem(ADDR_ITEM_STATIC_IP, (uint8_t*)config_static_ip, sizeof(config_static_ip));
  dxl_slave.addControlItem(ADDR_ITEM_ASSIGNED_IP, (uint8_t*)config_assigned_ip, sizeof(config_assigned_ip));
  dxl_slave.addControlItem(ADDR_MICRO_XRCE_DDS_AGENT_IP, (uint8_t*)config_u_xrce_agent_ip, sizeof(config_u_xrce_agent_ip));
  dxl_slave.addControlItem(ADDR_MICRO_XRCE_DDS_AGENT_PORT, config_u_xrce_agent_port);

  // Add interrupt callback functions to process read/write packet.
  dxl_slave.setReadCallbackFunc(read_callback_func);
  dxl_slave.setWriteCallbackFunc(write_callback_func);
}

void getConfigFromEEPROM()
{
  // EEPROM.begin(150);

  // EEPROM.get(ADDR_DXL_SLAVE_BAUDRATE, config_dxl_slave_baudrate_idx);
  // if(config_dxl_slave_baudrate_idx != 0xFF){
  //   dxl_slave_baudrate = getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx);
  //   if(dxl_slave_baudrate == 0){
  //     dxl_slave_baudrate = 1000000;
  //     config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);  
  //   }
  // }else{
  //   config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);
  //   if(config_dxl_slave_baudrate_idx == 0xFF){
  //     dxl_slave_baudrate = 1000000;
  //     config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);  
  //   }
  // }

  EEPROM.get(ADDR_ROS2_NODE_NAME, config_ros2_node_name);
  if(config_ros2_node_name[0] != 0xFF){
    p_ros2_node_name = (const char*)config_ros2_node_name;
  }else{
    strncpy(config_ros2_node_name, p_ros2_node_name, sizeof(config_ros2_node_name));
  }

  EEPROM.get(ADDR_AUTO_SCAN_START_ID, config_start_id_to_scan);
  if(config_start_id_to_scan != 0xFF){
    start_id_to_scan = config_start_id_to_scan;
  }else{
    config_start_id_to_scan = start_id_to_scan;
  }

  EEPROM.get(ADDR_AUTO_SCAN_END_ID, config_end_id_to_scan);
  if(config_end_id_to_scan != 0xFF){
    end_id_to_scan = config_end_id_to_scan;
  }else{
    config_end_id_to_scan = end_id_to_scan;
  }

  EEPROM.get(ADDR_AUTO_SCAN_INTERVAL_MS, config_auto_scan_interval_ms);
  if(config_auto_scan_interval_ms != 0xFFFFFFFF){
    auto_scan_interval_ms = config_auto_scan_interval_ms;
  }else{
    config_auto_scan_interval_ms = auto_scan_interval_ms;
  }

  EEPROM.get(ADDR_DXL_MASTER_BAUDRATE, config_dxl_master_baudrate_idx);
  if(config_dxl_master_baudrate_idx != 0xFF){
    dxl_master_baudrate = getBaudrateValueFromIndex(config_dxl_master_baudrate_idx);
    if(dxl_master_baudrate == 0){
      dxl_master_baudrate = 1000000;
      config_dxl_master_baudrate_idx = getBaudrateIndexFromValue(dxl_master_baudrate);  
    }
  }else{
    config_dxl_master_baudrate_idx = getBaudrateIndexFromValue(dxl_master_baudrate);
    if(config_dxl_master_baudrate_idx == 0xFF){
      dxl_master_baudrate = 1000000;
      config_dxl_master_baudrate_idx = getBaudrateIndexFromValue(dxl_master_baudrate);  
    }
  }

  EEPROM.get(ADDR_DXL_MASTER_PROTOCOL, config_dxl_master_protocol_ver);
  if(config_dxl_master_protocol_ver != 0xFF){
    dxl_master_protocol_ver = config_dxl_master_protocol_ver==1?1.0:2.0;
  }
  config_dxl_master_protocol_ver = dxl_master_protocol_ver==1.0?1:2;

  EEPROM.get(ADDR_ITEM_DHCP_ENABLE, config_dhcp_enable);
  if(config_dhcp_enable != 0xFF){
    dhcp_enable = config_dhcp_enable;
  }else{
    config_dhcp_enable = dhcp_enable;
  }

  EEPROM.get(ADDR_ITEM_MAC_ADDR, config_mac_addr);
  if(config_mac_addr[0] != 0xFF){
    p_mac = (const uint8_t*)config_mac_addr;
  }else{
    memcpy(config_mac_addr, p_mac, sizeof(config_mac_addr));
  }

  EEPROM.get(ADDR_ITEM_STATIC_IP, config_static_ip);
  if(config_static_ip[0] != 0xFF){
    p_static_ip = (const char*)config_static_ip;
    static_ip.fromString(p_static_ip);
  }else{
    strncpy(config_static_ip, p_static_ip, sizeof(config_static_ip));
  }

  EEPROM.get(ADDR_MICRO_XRCE_DDS_AGENT_IP, config_u_xrce_agent_ip);
  if(config_u_xrce_agent_ip[0] != 0xFF){
    p_agent_ip = (const char*)config_u_xrce_agent_ip;
  }else{
    strncpy(config_u_xrce_agent_ip, p_agent_ip, sizeof(config_u_xrce_agent_ip));
  }

  EEPROM.get(ADDR_MICRO_XRCE_DDS_AGENT_PORT, config_u_xrce_agent_port);
  if(config_u_xrce_agent_port != 0xFFFF){
    agent_port = config_u_xrce_agent_port;
  }else{
    config_u_xrce_agent_port = agent_port;
  }
}

void runDXLSlaveToConfig()
{
  dxl_slave.processPacket();
}

// Function to update data according to master's read request.
// If you use dxl_err_code, you can send the packet's error code to the Master.
// See the DYNAMIXEL Protocol documentation for this. 
// (http://emanual.robotis.com/docs/kr/dxl/protocol2/)
void read_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;

  switch(item_addr)
  {
    // case ADDR_DXL_SLAVE_BAUDRATE:
    //   config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);
    //   break;
    case ADDR_DXL_MASTER_BAUDRATE:
      config_dxl_master_baudrate_idx = getBaudrateIndexFromValue(dxl_master_baudrate);
      break;      
  }

}

// Function to update data according to master write request.
// If you use dxl_err_code, you can send the packet's error code to the Master.
// See the DYNAMIXEL Protocol documentation for this. 
// (http://emanual.robotis.com/docs/kr/dxl/protocol2/)
void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;

  switch(item_addr)
  {
    // case ADDR_DXL_SLAVE_BAUDRATE:
    //   if(getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx) != 0){
    //     EEPROM.put(ADDR_DXL_SLAVE_BAUDRATE, config_dxl_slave_baudrate_idx);
    //     dxl_slave_baudrate = getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx);
    //     // dxl_slave_port.begin(dxl_slave_baudrate);
    //   }else{
    //     dxl_err_code = DXL2_0_ERR_DATA_RANGE;
    //   }
    //   break;

    case ADDR_ROS2_NODE_NAME:
      EEPROM.put(ADDR_ROS2_NODE_NAME, config_ros2_node_name);
      break;

    case ADDR_AUTO_SCAN_START_ID:
      if(XELNetworkMaster::initScan(config_start_id_to_scan, end_id_to_scan, auto_scan_interval_ms) == true){
        EEPROM.put(ADDR_AUTO_SCAN_START_ID, config_start_id_to_scan);
        start_id_to_scan = config_start_id_to_scan;
      }
      break;

    case ADDR_AUTO_SCAN_END_ID:
      if(XELNetworkMaster::initScan(start_id_to_scan, config_end_id_to_scan, auto_scan_interval_ms) == true){
        EEPROM.put(ADDR_AUTO_SCAN_END_ID, config_end_id_to_scan);
        end_id_to_scan = config_end_id_to_scan;
      }
      break;
    
    case ADDR_AUTO_SCAN_INTERVAL_MS:
      EEPROM.put(ADDR_AUTO_SCAN_INTERVAL_MS, config_auto_scan_interval_ms);
      auto_scan_interval_ms = config_auto_scan_interval_ms;
      XELNetworkMaster::initScan(start_id_to_scan, end_id_to_scan, auto_scan_interval_ms);
      break;

    case ADDR_DXL_MASTER_BAUDRATE:
      if(getBaudrateValueFromIndex(config_dxl_master_baudrate_idx) != 0){
        EEPROM.put(ADDR_DXL_MASTER_BAUDRATE, config_dxl_master_baudrate_idx);
        dxl_master_baudrate = getBaudrateValueFromIndex(config_dxl_master_baudrate_idx);
        XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver);
      }else{
        dxl_err_code = DXL2_0_ERR_DATA_RANGE;
      }
      break;

    case ADDR_DXL_MASTER_PROTOCOL:
      if(config_dxl_master_protocol_ver != 1 && config_dxl_master_protocol_ver != 2){
        config_dxl_master_protocol_ver = dxl_master_protocol_ver==1.0?1:2;
        dxl_err_code = DXL2_0_ERR_DATA_RANGE;
      }else{
        EEPROM.put(ADDR_DXL_MASTER_PROTOCOL, config_dxl_master_protocol_ver);
        dxl_master_protocol_ver = config_dxl_master_protocol_ver==1?1.0:2.0;
        XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver);
      }
      break;

    case ADDR_ITEM_DHCP_ENABLE:
      EEPROM.put(ADDR_ITEM_DHCP_ENABLE, config_dhcp_enable);
      break;

    case ADDR_ITEM_MAC_ADDR:
      EEPROM.put(ADDR_ITEM_MAC_ADDR, config_mac_addr);
      break;

    case ADDR_ITEM_STATIC_IP:
      EEPROM.put(ADDR_ITEM_STATIC_IP, config_static_ip);
      break;

    case ADDR_MICRO_XRCE_DDS_AGENT_IP:
      EEPROM.put(ADDR_MICRO_XRCE_DDS_AGENT_IP, config_u_xrce_agent_ip);
      break;

    case ADDR_MICRO_XRCE_DDS_AGENT_PORT:
      EEPROM.put(ADDR_MICRO_XRCE_DDS_AGENT_PORT, config_u_xrce_agent_port);
      break;
  }
}




const uint16_t BUFFER_SIZE = 1024;
uint8_t buffer[BUFFER_SIZE];

void serial2DXL()
{
  static DYNAMIXEL::SerialPortHandler* p_dxl_port;  
  uint16_t len, i;
 
  p_dxl_port = getMasterPortHandler();
  if(p_dxl_port == nullptr){
    return;
  }

  uint32_t usb_baud = Serial.baud();

  if(usb_baud != p_dxl_port->getBaud())
  {
    p_dxl_port->begin(usb_baud);
  }

  len = Serial.available();
  if(len > 0){
    if(len > BUFFER_SIZE){
      len = BUFFER_SIZE;
    }
    for(i=0; i<len; i++){
      buffer[i] = Serial.read();
    }
    len = p_dxl_port->write(buffer, len);
  }

  len = p_dxl_port->available(); 
  if(len > 0){
    if(len > BUFFER_SIZE){
      len = BUFFER_SIZE;
    }
    for(i=0; i<len; i++){
      buffer[i] = p_dxl_port->read();
    }
    len = Serial.write(buffer, len);
  }
}

void checkButtonAndChangeMode()
{
  static DYNAMIXEL::SerialPortHandler* p_dxl_port;
  uint32_t pre_time;
  uint32_t button_pressed_time;

  pre_time = millis();
  while(digitalRead(USER_BTN) == HIGH)
  {
    // LED
    if(millis() - pre_time_led >= 50){
      pre_time_led = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
  button_pressed_time = millis()-pre_time;
  
  if(button_pressed_time >= 1000 
  && pre_op_mode == OP_XELNETWORK){
    p_dxl_port = getMasterPortHandler();
    if(p_dxl_port == nullptr){
      return;
    }    
    op_mode = OP_BYPASS_USB_DXL;
    enable_auto_reset = false;
    DEBUG_PRINTLN("\t [INFO] Entered BYPASS Mode");
    pre_op_mode = op_mode;    
  }else if(button_pressed_time > 100 && button_pressed_time < 1000 
  && pre_op_mode == OP_BYPASS_USB_DXL){
    p_dxl_port = getMasterPortHandler();
    if(p_dxl_port == nullptr){
      return;
    }
    op_mode = OP_XELNETWORK;
    enable_auto_reset = true;
    DEBUG_PRINTLN("\t [INFO] Entered XELNetwork Plug and Play Mode");
    pre_op_mode = op_mode;
  }
}


void printErrAndLED(const char* p_messesge, uint32_t interval_ms)
{
  if(millis() - pre_time_led >= interval_ms){
    pre_time_led = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    DEBUG_PRINTLN(p_messesge);
  }
}

