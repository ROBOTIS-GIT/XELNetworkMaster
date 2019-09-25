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

// Communication port setting for communication with ROS2.
#include <WiFi.h>
#include <WiFiUdp.h>
const char* p_ssid = "YOUR_SSID";
const char* p_ssid_pw = "YOUR_SSID_PASSWORD";

// Definition of information for connecting to MICRO-XRCE-DDS.
const char* p_ros2_node_name = "xelnetwork";
const char* p_agent_ip = "AGENT_IP_ADDRESS";
uint16_t agent_port = 2019; //AGENT port number

// Settings to communicate with other SLAVE devices (SensorXEL, etc.)
// by acting as master of the DYNAMIXEL protocol.
HardwareSerial& DXL_MASTER_SERIAL = Serial2;
const int DXL_MASTER_DIR_PIN = 15;
uint32_t dxl_master_baudrate = 57600;
float dxl_master_protocol_ver = 2.0;
uint32_t dxl_slave_baudrate = 1000000;

// Interval time setting to scan slave devices periodically for Plug And Play.
// If you set the parameter to 0, scans only at begin time and no longer scans.
uint32_t auto_scan_interval_ms = 200;
// The Scan function pings only one ID at a set interval.
// Therefore, by setting the range of IDs to scan,
// you can scan the same ID in shorter periods.
uint8_t start_id_to_scan = 0;
uint8_t end_id_to_scan = 20;


WiFiUDP udp;

void setup() {
  // put your setup code here, to run once:
  getConfigFromEEPROM();
  beginDXLSlaveToConfig(dxl_slave_baudrate);

  WiFi.begin(p_ssid, p_ssid_pw);
  while(WiFi.status() != WL_CONNECTED){
    runDXLSlaveToConfig();
  }

  XELNetworkMaster::initDXLMaster(DXL_MASTER_SERIAL, DXL_MASTER_DIR_PIN);
  XELNetworkMaster::initROS2(udp, p_agent_ip, agent_port);
  XELNetworkMaster::initScan(start_id_to_scan, end_id_to_scan, auto_scan_interval_ms);
  XELNetworkMaster::initNode(p_ros2_node_name);
  XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver);
}

void loop() {
  // put your main code here, to run repeatedly:
  runDXLSlaveToConfig();

  XELNetworkMaster::run();
}





// Slave and EEPROM setting to allow to set/get WiFi and Agent IP settings 
// through the DYNAMIXEL protocol.
#include <EEPROM.h>
HardwareSerial& DXL_SLAVE_SERIAL = Serial;
const uint16_t COMMXEL_W_MODEL_NUM = 0x5042;
DYNAMIXEL::SerialPortHandler dxl_slave_port(DXL_SLAVE_SERIAL);
DYNAMIXEL::Slave dxl_slave(dxl_slave_port, COMMXEL_W_MODEL_NUM);

enum CommXELItemAddr{
  // ADDR_MODEL_NUMBER    = 0, //Default setting in the Slave class.
  // ADDR_FIRMWARE_VER    = 6, //Default setting in the Slave class.
  // ADDR_ID              = 7, //Default setting in the Slave class.
  ADDR_DXL_SLAVE_BAUDRATE        = 8,   //1byte
  ADDR_ROS2_NODE_NAME            = 10,  //32byte char
  ADDR_AUTO_SCAN_START_ID        = 50,  //uint8_t
  ADDR_AUTO_SCAN_END_ID          = 51,  //uint8_t
  ADDR_AUTO_SCAN_INTERVAL_MS     = 52,  //uint32_t
  ADDR_DXL_MASTER_BAUDRATE       = 58,  //uint32_t
  ADDR_DXL_MASTER_PROTOCOL       = 59,  //uint8_t
  ADDR_WIFI_SSID                 = 60,  //32byte char
  ADDR_WIFI_SSID_PW              = 92,  //32byte char
  ADDR_MICRO_XRCE_DDS_AGENT_IP   = 124, //12byte char "xxx.xxx.xxx"
  ADDR_MICRO_XRCE_DDS_AGENT_PORT = 136  //uint16_t
};

static uint8_t config_dxl_slave_baudrate_idx;
static char config_ros2_node_name[32];
static uint8_t config_start_id_to_scan;
static uint8_t config_end_id_to_scan;
static uint32_t config_auto_scan_interval_ms;
static uint8_t config_dxl_master_baudrate_idx;
static uint8_t config_dxl_master_protocol_ver;
static char config_wifi_ssid[32];
static char config_wifi_ssid_pw[32];
static char config_u_xrce_agent_ip[12];
static uint16_t config_u_xrce_agent_port;


void beginDXLSlaveToConfig(uint32_t port_baud)
{
  dxl_slave_port.begin(port_baud);

  dxl_slave.setFirmwareVersion(1);
  dxl_slave.setID(200);

  dxl_slave.addControlItem(ADDR_DXL_SLAVE_BAUDRATE, (uint8_t*)&config_dxl_slave_baudrate_idx, sizeof(config_dxl_slave_baudrate_idx));
  dxl_slave.addControlItem(ADDR_ROS2_NODE_NAME, (uint8_t*)config_ros2_node_name, sizeof(config_ros2_node_name));
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_START_ID, config_start_id_to_scan);
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_END_ID, config_end_id_to_scan);
  dxl_slave.addControlItem(ADDR_AUTO_SCAN_INTERVAL_MS, config_auto_scan_interval_ms);
  dxl_slave.addControlItem(ADDR_DXL_MASTER_BAUDRATE, config_dxl_master_baudrate_idx);
  dxl_slave.addControlItem(ADDR_DXL_MASTER_PROTOCOL, config_dxl_master_protocol_ver);
  dxl_slave.addControlItem(ADDR_WIFI_SSID, (uint8_t*)config_wifi_ssid, sizeof(config_wifi_ssid));
  dxl_slave.addControlItem(ADDR_WIFI_SSID_PW, (uint8_t*)config_wifi_ssid_pw, sizeof(config_wifi_ssid_pw));
  dxl_slave.addControlItem(ADDR_MICRO_XRCE_DDS_AGENT_IP, (uint8_t*)config_u_xrce_agent_ip, sizeof(config_u_xrce_agent_ip));
  dxl_slave.addControlItem(ADDR_MICRO_XRCE_DDS_AGENT_PORT, config_u_xrce_agent_port);

  // Add interrupt callback functions to process read/write packet.
  dxl_slave.setReadCallbackFunc(read_callback_func);
  dxl_slave.setWriteCallbackFunc(write_callback_func);
}

void getConfigFromEEPROM()
{
  EEPROM.begin(150);

  if(EEPROM.readBytes(ADDR_DXL_SLAVE_BAUDRATE, (void*)&config_dxl_slave_baudrate_idx, sizeof(config_dxl_slave_baudrate_idx)) != 0){
    if(config_dxl_slave_baudrate_idx != 0xFF){
      dxl_slave_baudrate = getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx);
      if(dxl_slave_baudrate == 0){
        dxl_slave_baudrate = 1000000;
        config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);  
      }
    }else{
      config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);
      if(config_dxl_slave_baudrate_idx == 0xFF){
        dxl_slave_baudrate = 1000000;
        config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_baudrate);  
      }
    }
  }

  if(EEPROM.readBytes(ADDR_ROS2_NODE_NAME, (void*)config_ros2_node_name, sizeof(config_ros2_node_name)) != 0){
    if(config_ros2_node_name[0] != 0xFF){
      p_ros2_node_name = (const char*)config_ros2_node_name;
    }else{
      strncpy(config_ros2_node_name, p_ros2_node_name, sizeof(config_ros2_node_name));
    }
  }

  if(EEPROM.readBytes(ADDR_AUTO_SCAN_START_ID, (void*)&config_start_id_to_scan, sizeof(config_start_id_to_scan)) != 0){
    if(config_start_id_to_scan != 0xFF){
      start_id_to_scan = config_start_id_to_scan;
    }else{
      config_start_id_to_scan = start_id_to_scan;
    }
  }

  if(EEPROM.readBytes(ADDR_AUTO_SCAN_END_ID, (void*)&config_end_id_to_scan, sizeof(config_end_id_to_scan)) != 0){
    if(config_end_id_to_scan != 0xFF){
      end_id_to_scan = config_end_id_to_scan;
    }else{
      config_end_id_to_scan = end_id_to_scan;
    }
  }

  if(EEPROM.readBytes(ADDR_AUTO_SCAN_INTERVAL_MS, (void*)&config_auto_scan_interval_ms, sizeof(config_auto_scan_interval_ms)) != 0){
    if(config_auto_scan_interval_ms != 0xFFFFFFFF){
      auto_scan_interval_ms = config_auto_scan_interval_ms;
    }else{
      config_auto_scan_interval_ms = auto_scan_interval_ms;
    }
  }

  if(EEPROM.readBytes(ADDR_DXL_MASTER_BAUDRATE, (void*)&config_dxl_master_baudrate_idx, sizeof(config_dxl_master_baudrate_idx)) != 0){
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

  }

  if(EEPROM.readBytes(ADDR_DXL_MASTER_PROTOCOL, (void*)&config_dxl_master_protocol_ver, sizeof(config_dxl_master_protocol_ver)) != 0){
    if(config_dxl_master_protocol_ver != 0xFF){
      dxl_master_protocol_ver = config_dxl_master_protocol_ver==1?1.0:2.0;
    }
    config_dxl_master_protocol_ver = dxl_master_protocol_ver==1.0?1:2;
  }

  if(EEPROM.readBytes(ADDR_WIFI_SSID, (void*)config_wifi_ssid, sizeof(config_wifi_ssid)) != 0){
    if(config_wifi_ssid[0] != 0xFF){
      p_ssid = (const char*)config_wifi_ssid;
    }else{
      strncpy(config_wifi_ssid, p_ssid, sizeof(config_wifi_ssid));
    }
  }

  if(EEPROM.readBytes(ADDR_WIFI_SSID_PW, (void*)config_wifi_ssid_pw, sizeof(config_wifi_ssid_pw)) != 0){
    if(config_wifi_ssid_pw[0] != 0xFF){
      p_ssid_pw = (const char*)config_wifi_ssid_pw;
    }else{
      strncpy(config_wifi_ssid_pw, p_ssid_pw, sizeof(config_wifi_ssid_pw));
    }
  }

  if(EEPROM.readBytes(ADDR_MICRO_XRCE_DDS_AGENT_IP, (void*)config_u_xrce_agent_ip, sizeof(config_u_xrce_agent_ip)) != 0){
    if(config_u_xrce_agent_ip[0] != 0xFF){
      p_agent_ip = (const char*)config_u_xrce_agent_ip;
    }else{
      strncpy(config_u_xrce_agent_ip, p_agent_ip, sizeof(config_u_xrce_agent_ip));
    }
  }

  if(EEPROM.readBytes(ADDR_MICRO_XRCE_DDS_AGENT_PORT, (void*)&config_u_xrce_agent_port, sizeof(config_u_xrce_agent_port)) != 0){
    if(config_u_xrce_agent_port != 0xFFFF){
      agent_port = config_u_xrce_agent_port;
    }else{
      config_u_xrce_agent_port = agent_port;
    }
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
    case ADDR_DXL_SLAVE_BAUDRATE:
      config_dxl_slave_baudrate_idx = getBaudrateIndexFromValue(dxl_slave_port.getBaud());
      break;
    case ADDR_DXL_SLAVE_BAUDRATE:
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
    case ADDR_DXL_SLAVE_BAUDRATE:
      if(getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx) != 0){
        EEPROM.writeBytes(ADDR_DXL_SLAVE_BAUDRATE, (const void*)&config_dxl_slave_baudrate_idx, sizeof(config_dxl_slave_baudrate_idx));
        dxl_slave_baudrate = getBaudrateValueFromIndex(config_dxl_slave_baudrate_idx);
        dxl_slave_port.begin(dxl_slave_baudrate);
        EEPROM.commit();
      }else{
        dxl_err_code = DXL_ERR_DATA_RANGE;
      }
      break;

    case ADDR_ROS2_NODE_NAME:
      EEPROM.writeBytes(ADDR_ROS2_NODE_NAME, (const void*)config_ros2_node_name, sizeof(config_ros2_node_name));
      EEPROM.commit();
      break;

    case ADDR_AUTO_SCAN_START_ID:
      if(XELNetworkMaster::initScan(config_start_id_to_scan, end_id_to_scan, auto_scan_interval_ms) == true){
        EEPROM.writeBytes(ADDR_AUTO_SCAN_START_ID, (const void*)&config_start_id_to_scan, sizeof(config_start_id_to_scan));
        start_id_to_scan = config_start_id_to_scan;
        EEPROM.commit();
      }
      break;

    case ADDR_AUTO_SCAN_END_ID:
      if(XELNetworkMaster::initScan(start_id_to_scan, config_end_id_to_scan, auto_scan_interval_ms) == true){
        EEPROM.writeBytes(ADDR_AUTO_SCAN_END_ID, (const void*)&config_end_id_to_scan, sizeof(config_end_id_to_scan));
        end_id_to_scan = config_end_id_to_scan;
        EEPROM.commit();
      }
      break;
    
    case ADDR_AUTO_SCAN_INTERVAL_MS:
      EEPROM.writeBytes(ADDR_AUTO_SCAN_INTERVAL_MS, (const void*)&config_auto_scan_interval_ms, sizeof(config_auto_scan_interval_ms));
      auto_scan_interval_ms = config_auto_scan_interval_ms;
      XELNetworkMaster::initScan(start_id_to_scan, end_id_to_scan, auto_scan_interval_ms);
      EEPROM.commit();
      break;

    case ADDR_DXL_MASTER_BAUDRATE:
      if(getBaudrateValueFromIndex(config_dxl_master_baudrate_idx) != 0){
        EEPROM.writeBytes(ADDR_DXL_SLAVE_BAUDRATE, (const void*)&config_dxl_master_baudrate_idx, sizeof(config_dxl_master_baudrate_idx));
        dxl_master_baudrate = getBaudrateValueFromIndex(config_dxl_master_baudrate_idx);
        XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver);
        EEPROM.commit();
      }else{
        dxl_err_code = DXL_ERR_DATA_RANGE;
      }    

    case ADDR_DXL_MASTER_PROTOCOL:
      if(config_dxl_master_protocol_ver != 1 && config_dxl_master_protocol_ver != 2){
        config_dxl_master_protocol_ver = dxl_master_protocol_ver==1.0?1:2;
        dxl_err_code = DXL_ERR_DATA_RANGE;
      }else{
        EEPROM.writeBytes(ADDR_DXL_MASTER_PROTOCOL, (const void*)&config_dxl_master_protocol_ver, sizeof(config_dxl_master_protocol_ver));    
        dxl_master_protocol_ver = config_dxl_master_protocol_ver==1?1.0:2.0;
        XELNetworkMaster::begin(dxl_master_baudrate, dxl_master_protocol_ver);
        EEPROM.commit();
      }
      break;

    case ADDR_WIFI_SSID:
      EEPROM.writeBytes(ADDR_WIFI_SSID, (const void*)config_wifi_ssid, sizeof(config_wifi_ssid));
      EEPROM.commit();
      break;

    case ADDR_WIFI_SSID_PW:
      EEPROM.writeBytes(ADDR_WIFI_SSID_PW, (const void*)config_wifi_ssid_pw, sizeof(config_wifi_ssid_pw));
      EEPROM.commit();
      break;

    case ADDR_MICRO_XRCE_DDS_AGENT_IP:
      EEPROM.writeBytes(ADDR_MICRO_XRCE_DDS_AGENT_IP, (const void*)config_u_xrce_agent_ip, sizeof(config_u_xrce_agent_ip));
      EEPROM.commit();
      break;

    case ADDR_MICRO_XRCE_DDS_AGENT_PORT:
      EEPROM.writeBytes(ADDR_MICRO_XRCE_DDS_AGENT_PORT, (const void*)&config_u_xrce_agent_port, sizeof(config_u_xrce_agent_port));
      EEPROM.commit();
      break;
  }
}
