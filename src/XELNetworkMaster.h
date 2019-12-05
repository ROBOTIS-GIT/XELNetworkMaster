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

#ifndef XEL_NETWORK_MASTER_H_
#define XEL_NETWORK_MASTER_H_

#include "utility/XELNetworkCommon.h"

class Esp32SerialPortHandler : public DYNAMIXEL::SerialPortHandler
{
  public:
    Esp32SerialPortHandler(HardwareSerial& port, const int dir_pin = -1)
    : SerialPortHandler(port, dir_pin), port_(port), dir_pin_(dir_pin), baud_(1000000)
    {}

  virtual void begin(unsigned long baud) override
  {
    if(getOpenState() == true){
      port_.updateBaudRate(baud);
    }else{
      port_.begin(baud);
      if(dir_pin_ != -1){
        pinMode(dir_pin_, OUTPUT);
        digitalWrite(dir_pin_, LOW);
        while(digitalRead(dir_pin_) != LOW);
      }
      setOpenState(true);
    }
    baud_ = baud;
  }

  virtual unsigned long getBaud() const override
  {
    return baud_;
  }

  private:
    HardwareSerial& port_;
    const int dir_pin_;
    unsigned long baud_;
    bool is_begin_;
};

DYNAMIXEL::SerialPortHandler* getMasterPortHandler();

namespace XELNetworkMaster{

bool initDXLMaster(HardwareSerial& dxl_port_serial, int dxl_dir_pin);
bool initROS2(Stream& comm_instance);
bool initROS2(UDP& comm_instance, const char* p_agent_ip, uint16_t agent_port);
bool initROS2(Client& comm_instance, const char* p_agent_ip, uint16_t agent_port);
bool initNode(const char* node_name = "xelnetwork_comm");
bool initScan(uint8_t start_id, uint8_t end_id, uint32_t interval_ms = 200);
bool begin(uint32_t dxl_port_baud, float dxl_port_protocol_ver = 2.0);
void run();

}//namespace XELNetworkMaster

#endif /* XEL_NETWORK_MASTER_H_ */