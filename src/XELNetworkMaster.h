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

namespace XELNetworkMaster{

bool initDXLMaster(HardwareSerial& dxl_port_serial, int dxl_dir_pin);
bool initROS2(Stream& comm_instance);
bool initROS2(UDP& comm_instance, const char* p_agent_ip, uint16_t agent_port);
bool initROS2(Client& comm_instance, const char* p_agent_ip, uint16_t agent_port);
bool begin(uint32_t dxl_port_baud, float dxl_port_protocol_ver, uint32_t auto_scan_interval_ms = 200);
void run();

}//namespace XELNetworkMaster


#endif /* XEL_NETWORK_MASTER_H_ */