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

#include "XELNetworkMaster.h"

#include <ros2arduino.h>
#include "utility/XELNetworkCommon.h"
#include "utility/XELNetworkROS2Node.hpp"

const uint8_t CONNECTED_XEL_MAX = 10;
const uint8_t PING_RETRY_COUNT = 3;
const uint8_t DEFAULT_START_ID_TO_SCAN = 0;
const uint8_t DEFAULT_END_ID_TO_SCAN = 20;
const uint32_t DEFAULT_SCAN_INTERVAL_MS = 200;

enum XELStatusIdx{
  NOT_CONNECTTED = 0,
  NEW_CONNECTION,
  RUNNING
};

typedef struct XELStatus{
  uint8_t previous;
  uint8_t present;
} XELStatus_t;

typedef struct XELInfo{
  uint8_t id;
  uint8_t topic_cnt;
  XELStatus_t status;
  TopicInfo_t topic_info[SLAVE_TOPIC_ITEM_MAX];
} XELInfo_t;

static XELNetworkROS2Node *p_ros2_node;
static DYNAMIXEL::SerialPortHandler *p_dxl_port;
static DYNAMIXEL::Master *p_dxl_master;

static XELInfo_t xel_table[CONNECTED_XEL_MAX];
static uint8_t connected_xel_cnt;
static uint8_t limited_start_id_to_scan = DEFAULT_START_ID_TO_SCAN;
static uint8_t limited_end_id_to_scan = DEFAULT_END_ID_TO_SCAN;
static uint32_t auto_scan_interval_ms = DEFAULT_SCAN_INTERVAL_MS;
static bool is_init_dxl_master = false;
static bool is_init_ros2 = false;

static void scanXELsWhenBegin();
static void scanXELEveryInterval();
static bool createNewTopicWithXELItem(TopicInfo_t& topic_item);
static ros2::CallbackFunc getCallbackFromTopicItemId(uint8_t topic_item_id);


DYNAMIXEL::SerialPortHandler* getMasterPortHandler()
{
  return p_dxl_port;
}


bool XELNetworkMaster::initDXLMaster(HardwareSerial& dxl_port_serial, int dxl_dir_pin)
{
  static Esp32SerialPortHandler dxl_port(dxl_port_serial, dxl_dir_pin);
  static DYNAMIXEL::Master dxl;

  dxl_port.begin(1000000);
  dxl.setPort((DXLPortHandler*)&dxl_port);
  
  p_dxl_port = &dxl_port;
  p_dxl_master = &dxl;
  
  is_init_dxl_master = true;

  return is_init_dxl_master;
}

bool XELNetworkMaster::initROS2(Stream& comm_instance)
{
  if(is_init_ros2 == false)
    is_init_ros2 = ros2::init(&comm_instance);

  return is_init_ros2;
}

bool XELNetworkMaster::initROS2(UDP& comm_instance, const char* p_agent_ip, uint16_t agent_port)
{
  if(is_init_ros2 == false)
    is_init_ros2 = ros2::init(&comm_instance, p_agent_ip, agent_port);
  
  return is_init_ros2;
}

bool XELNetworkMaster::initROS2(Client& comm_instance, const char* p_agent_ip, uint16_t agent_port)
{
  if(is_init_ros2 == false)
    is_init_ros2 = ros2::init(&comm_instance, p_agent_ip, agent_port);

  return is_init_ros2;
}

bool XELNetworkMaster::initScan(uint8_t start_id, uint8_t end_id, uint32_t interval_ms)
{
  auto_scan_interval_ms = interval_ms;

  if(start_id > 253 || end_id > 253 || start_id > end_id)
    return false;

  limited_start_id_to_scan = start_id;
  limited_end_id_to_scan = end_id;

  return true;
}

bool XELNetworkMaster::initNode(const char* node_name)
{
  bool ret = false;

  if(p_ros2_node == nullptr){
    p_ros2_node = new XELNetworkROS2Node(node_name);

    if(p_ros2_node!=nullptr){
      ret = p_ros2_node->getNodeRegisteredState();
    }    
  }else{
    p_ros2_node->recreateNode();
    ret = p_ros2_node->getNodeRegisteredState();
  }
  
  return ret;
}

bool XELNetworkMaster::begin(uint32_t dxl_port_baud, float dxl_port_protocol_ver)
{
  if(is_init_dxl_master == false || is_init_ros2 == false){
    return false;
  }

  p_dxl_port->begin(dxl_port_baud);

  if(p_dxl_master->setPortProtocolVersion(dxl_port_protocol_ver) == false)
    p_dxl_master->setPortProtocolVersion(2.0);

  if(p_dxl_master->getPortProtocolVersion() == 2.0){
    if(limited_start_id_to_scan > 252)
      limited_start_id_to_scan = 252;
    if(limited_end_id_to_scan > 252)
      limited_end_id_to_scan = 252;
  }

  scanXELsWhenBegin();

  return true;
}

void XELNetworkMaster::run()
{
  uint8_t i, j;
  XELInfo_t* p_xel;

  scanXELEveryInterval(); 

  for(i = 0; i < CONNECTED_XEL_MAX; i++)
  {
    p_xel = &xel_table[i];
    switch(p_xel->status.present)
    {
      case NOT_CONNECTTED:
        if(p_xel->status.previous == RUNNING){
          for(j=0; j<p_xel->topic_cnt; j++)
          {
            p_ros2_node->deleteTopic(p_xel->topic_info[j]);
          }

          memset(p_xel, 0, sizeof(XELInfo_t));
          p_xel->status.previous = NOT_CONNECTTED;
          p_xel->status.present = NOT_CONNECTTED;
        }
        
        break;

      case NEW_CONNECTION:
        for(j=0; j<p_xel->topic_cnt; j++)
        {
          if(createNewTopicWithXELItem(p_xel->topic_info[j]) == true){
            p_xel->status.previous = p_xel->status.present;
            p_xel->status.present = RUNNING;
          }
        }
        break;

      case RUNNING:
        break;
    }
  }

  // Process ros2 node.
  ros2::spin(p_ros2_node);
} 








static void scanXELsWhenBegin()
{
  XelInfoFromPing_t ping_info;
  TopicItemHeader_t topic_item_header;
  XELInfo_t* p_xel;

  for(uint8_t i_id = limited_start_id_to_scan; i_id <= limited_end_id_to_scan && connected_xel_cnt < CONNECTED_XEL_MAX; i_id++)
  {
    if(p_dxl_master->ping(i_id, &ping_info, 1, 10) == 1){
      p_dxl_master->read(ping_info.id, 0, 2, (uint8_t*)&ping_info.model_number, sizeof(ping_info.model_number), 10);
      if(ping_info.model_number == XEL_NETWORK_SLAVE_MODEL_NUM){
        if(p_dxl_master->read(ping_info.id, SLAVE_TOPIC_ITEM_FIRST_ADDR, sizeof(TopicItemHeader_t), (uint8_t*)&topic_item_header, sizeof(TopicItemHeader_t), 10) == sizeof(TopicItemHeader_t)){
          p_xel = &xel_table[connected_xel_cnt++];
          p_xel->topic_cnt = 0;
          p_xel->id = ping_info.id;
          p_xel->status.previous = NOT_CONNECTTED;
          p_xel->status.present = NEW_CONNECTION;
          p_xel->topic_info[p_xel->topic_cnt].header = topic_item_header;
          p_xel->topic_info[p_xel->topic_cnt].xel_id = p_xel->id;
          p_xel->topic_info[p_xel->topic_cnt].callback_func = getCallbackFromTopicItemId(topic_item_header.id);
          p_xel->topic_cnt++;
          while(topic_item_header.next_item_addr != 0 && p_xel->topic_cnt < SLAVE_TOPIC_ITEM_MAX){
            if(p_dxl_master->read(ping_info.id, topic_item_header.next_item_addr, sizeof(TopicItemHeader_t), (uint8_t*)&topic_item_header, sizeof(TopicItemHeader_t), 10) == sizeof(TopicItemHeader_t)){
              p_xel->topic_info[p_xel->topic_cnt].header = topic_item_header;
              p_xel->topic_info[p_xel->topic_cnt].xel_id = p_xel->id;
              p_xel->topic_info[p_xel->topic_cnt].callback_func = getCallbackFromTopicItemId(topic_item_header.id);
              p_xel->topic_cnt++;
            }else{
              break;
            }
          }
        }
      }
    }
  }
}

static void scanXELEveryInterval()
{
  static uint8_t new_id = limited_start_id_to_scan;
  static uint32_t pre_time;
  static uint8_t table_idx_to_check = 0;
  XelInfoFromPing_t ping_info;
  XELInfo_t* p_xel;

  if(auto_scan_interval_ms == 0 || millis() - pre_time < auto_scan_interval_ms){
    return;
  }
  pre_time = millis();
  
  if(table_idx_to_check >= CONNECTED_XEL_MAX) {
    table_idx_to_check = 0;
  }

  p_xel = &xel_table[table_idx_to_check++];

  if (p_xel->status.present == RUNNING || p_xel->status.present == NEW_CONNECTION){
    bool ret = false;
    for(uint8_t i = 0; i < PING_RETRY_COUNT; i++)
    { 
      if (p_dxl_master->ping(p_xel->id, &ping_info, 1, 10) == 1){
        ret = true;
        break;
      }
    }

    if (ret == false) {
      p_xel->status.previous = p_xel->status.present;
      p_xel->status.present = NOT_CONNECTTED;
      connected_xel_cnt--;
    }
  }else{  //new id scan
    for(uint8_t i = 0; i < CONNECTED_XEL_MAX; i++)
    {
      if(xel_table[i].id == new_id
      &&(xel_table[i].status.present == RUNNING || xel_table[i].status.present == NEW_CONNECTION)){
        new_id++;
        return;
      }
    }

    TopicItemHeader_t topic_item_header;

    if(p_dxl_master->ping(new_id++, &ping_info, 1, 10) == 1){
      p_dxl_master->read(ping_info.id, 0, 2, (uint8_t*)&ping_info.model_number, sizeof(ping_info.model_number), 10);
      if(ping_info.model_number == XEL_NETWORK_SLAVE_MODEL_NUM){
        if(p_dxl_master->read(ping_info.id, SLAVE_TOPIC_ITEM_FIRST_ADDR, sizeof(TopicItemHeader_t), (uint8_t*)&topic_item_header, sizeof(TopicItemHeader_t), 10) == sizeof(TopicItemHeader_t)){
          p_xel->topic_cnt = 0;
          p_xel->id = ping_info.id;
          p_xel->status.previous = NOT_CONNECTTED;
          p_xel->status.present = NEW_CONNECTION;
          p_xel->topic_info[p_xel->topic_cnt].header = topic_item_header;
          p_xel->topic_info[p_xel->topic_cnt].xel_id = p_xel->id;
          p_xel->topic_info[p_xel->topic_cnt].callback_func = getCallbackFromTopicItemId(topic_item_header.id);
          p_xel->topic_cnt++;
          while(topic_item_header.next_item_addr != 0 && p_xel->topic_cnt < SLAVE_TOPIC_ITEM_MAX){
            if(p_dxl_master->read(ping_info.id, topic_item_header.next_item_addr, sizeof(TopicItemHeader_t), (uint8_t*)&topic_item_header, sizeof(TopicItemHeader_t), 10) == sizeof(TopicItemHeader_t)){
              p_xel->topic_info[p_xel->topic_cnt].header = topic_item_header;
              p_xel->topic_info[p_xel->topic_cnt].xel_id = p_xel->id;
              p_xel->topic_info[p_xel->topic_cnt].callback_func = getCallbackFromTopicItemId(topic_item_header.id);
              p_xel->topic_cnt++;
            }else{
              break;
            }
          }
        }
      }
    }
  }
}





/////////////////////////////////////////////////////////////
///// Message Callback
/////////////////////////////////////////////////////////////
static void callbackMsgStdBool(std_msgs::Bool* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  bool data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&msg->data, sizeof(msg->data));
  }
}

static void callbackMsgStdChar(std_msgs::Char* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  char data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdInt8(std_msgs::Int8* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  int8_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdInt16(std_msgs::Int16* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  int16_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdInt32(std_msgs::Int32* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  int32_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdInt64(std_msgs::Int64* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  int64_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdUint8(std_msgs::UInt8* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  uint8_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdUint16(std_msgs::UInt16* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  uint16_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdUint32(std_msgs::UInt32* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  uint32_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdUint64(std_msgs::UInt64* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  uint64_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdFloat32(std_msgs::Float32* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  float data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgStdFloat64(std_msgs::Float64* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  double data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->data = data;
    }
  }else{
    data = msg->data;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgGeoPoint(geometry_msgs::Point* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  geometry_msgs_Point_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->x = data.x;
      msg->y = data.y;
      msg->z = data.z;
    }
  }else{
    data.x = msg->x;
    data.y = msg->y;
    data.z = msg->z;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgGeoVector3(geometry_msgs::Vector3* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  geometry_msgs_Vector3_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->x = data.x;
      msg->y = data.y;
      msg->z = data.z;
    }
  }else{
    data.x = msg->x;
    data.y = msg->y;
    data.z = msg->z;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgGeoQuaternion(geometry_msgs::Quaternion* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  geometry_msgs_Quaternion_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->x = data.x;
      msg->y = data.y;
      msg->z = data.z;
      msg->w = data.w;
    }
  }else{
    data.x = msg->x;
    data.y = msg->y;
    data.z = msg->z;
    data.w = msg->w;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgGeoTwist(geometry_msgs::Twist* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  geometry_msgs_Twist_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->linear.x = data.linear.x;
      msg->linear.y = data.linear.y;
      msg->linear.z = data.linear.z;
      msg->angular.x = data.angular.x;
      msg->angular.y = data.angular.y;
      msg->angular.z = data.angular.z;      
    }
  }else{
    data.linear.x = msg->linear.x;
    data.linear.y = msg->linear.y;
    data.linear.z = msg->linear.z;
    data.angular.x = msg->angular.x;
    data.angular.y = msg->angular.y;
    data.angular.z = msg->angular.z;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}

static void callbackMsgGeoPose(geometry_msgs::Pose* msg, void* arg)
{
  TopicInfo_t *p_topic_info = (TopicInfo_t*)arg;
  uint16_t addr = p_topic_info->header.data_addr;
  uint16_t length = getSizeOfTopicType(p_topic_info->header.id);
  uint8_t xel_id = p_topic_info->xel_id;

  geometry_msgs_Pose_t data;

  if(p_topic_info->header.mode == TopicMode::PUB){
    if(p_dxl_master->read(xel_id, addr, length, (uint8_t*)&data, sizeof(data), 10) == length){
      msg->position.x = data.position.x;
      msg->position.y = data.position.y;
      msg->position.z = data.position.z;
      msg->orientation.x = data.orientation.x;
      msg->orientation.y = data.orientation.y;
      msg->orientation.z = data.orientation.z;   
      msg->orientation.w = data.orientation.w;
    }
  }else{
    data.position.x = msg->position.x;
    data.position.y = msg->position.y;
    data.position.z = msg->position.z;
    data.orientation.x = msg->orientation.x;
    data.orientation.y = msg->orientation.y;
    data.orientation.z = msg->orientation.z;
    data.orientation.w = msg->orientation.w;
    p_dxl_master->write(xel_id, addr, (const uint8_t*)&data, sizeof(data), 10);
  }
}


static ros2::CallbackFunc getCallbackFromTopicItemId(uint8_t topic_item_id)
{
  ros2::CallbackFunc p_callback = nullptr;

  switch(topic_item_id)
  {
    case STD_MSGS_BOOL_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdBool;
      break;

    case STD_MSGS_CHAR_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdChar;
      break;

    case STD_MSGS_INT8_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdInt8;
      break;

    case STD_MSGS_INT16_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdInt16;
      break;

    case STD_MSGS_INT32_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdInt32;
      break;

    case STD_MSGS_INT64_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdInt64;
      break;

    case STD_MSGS_UINT8_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdUint8;
      break;

    case STD_MSGS_UINT16_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdUint16;
      break;

    case STD_MSGS_UINT32_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdUint32;
      break;

    case STD_MSGS_UINT64_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdUint64;
      break;

    case STD_MSGS_FLOAT32_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdFloat32;
      break;

    case STD_MSGS_FLOAT64_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgStdFloat64;
      break;

    case GEOMETRY_MSGS_POINT_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgGeoPoint;
      break;

    case GEOMETRY_MSGS_VECTOR3_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgGeoVector3;
      break;

    case GEOMETRY_MSGS_QUATERNION_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgGeoQuaternion;
      break;

    case GEOMETRY_MSGS_TWIST_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgGeoTwist;
      break;

    case GEOMETRY_MSGS_POSE_TOPIC_ID:
      p_callback = (ros2::CallbackFunc)callbackMsgGeoPose;
      break;
  }

  return p_callback;
}


static bool createNewTopicWithXELItem(TopicInfo_t& topic_item)
{
  bool ret = false;

  if(p_ros2_node == nullptr){
    return false;
  }

  switch(topic_item.header.id)
  {
    case STD_MSGS_BOOL_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Bool>(topic_item);
      break;

    case STD_MSGS_CHAR_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Char>(topic_item);
      break;

    case STD_MSGS_INT8_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Int8>(topic_item);
      break;

    case STD_MSGS_INT16_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Int16>(topic_item);
      break;

    case STD_MSGS_INT32_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Int32>(topic_item);
      break;

    case STD_MSGS_INT64_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Int64>(topic_item);
      break;

    case STD_MSGS_UINT8_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::UInt8>(topic_item);
      break;

    case STD_MSGS_UINT16_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::UInt16>(topic_item);
      break;

    case STD_MSGS_UINT32_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::UInt32>(topic_item);
      break;

    case STD_MSGS_UINT64_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::UInt64>(topic_item);
      break;

    case STD_MSGS_FLOAT32_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Float32>(topic_item);
      break;

    case STD_MSGS_FLOAT64_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<std_msgs::Float64>(topic_item);
      break;

    case GEOMETRY_MSGS_POINT_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<geometry_msgs::Point>(topic_item);
      break;

    case GEOMETRY_MSGS_VECTOR3_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<geometry_msgs::Vector3>(topic_item);
      break;

    case GEOMETRY_MSGS_QUATERNION_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<geometry_msgs::Quaternion>(topic_item);
      break;

    case GEOMETRY_MSGS_TWIST_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<geometry_msgs::Twist>(topic_item);
      break;

    case GEOMETRY_MSGS_POSE_TOPIC_ID:
      ret = p_ros2_node->createNewTopic<geometry_msgs::Pose>(topic_item);
      break;      
  }

  return ret;
}

