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
const char* p_agent_ip = "AGENT_IP_ADDRESS";
uint16_t agent_port = 2019; //AGENT port number

// Settings to communicate with other SLAVE devices (SensorXEL, etc.)
// by acting as master of the DYNAMIXEL protocol.
HardwareSerial& DXL_MASTER_SERIAL = Serial1;
const int DXL_MASTER_DIR_PIN = 15;
uint32_t dxl_master_baud_rate = 57600;
float dxl_master_protocol_ver = 2.0;

// Interval time setting to scan slave devices periodically for Plug And Play.
// If you set the parameter to 0xFFFFFFFF, scans only at begin time and no longer scans.
uint32_t auto_scan_interval_ms = 200;

WiFiUDP udp;

void setup() {
  // put your setup code here, to run once:
  WiFi.begin(p_ssid, p_ssid_pw);
  while(WiFi.status() != WL_CONNECTED);

  XELNetworkMaster::initDXLMaster(DXL_MASTER_SERIAL, DXL_MASTER_DIR_PIN);
  XELNetworkMaster::initROS2(udp, p_agent_ip, agent_port);
  XELNetworkMaster::begin(dxl_master_baud_rate, dxl_master_protocol_ver, auto_scan_interval_ms);
}

void loop() {
  // put your main code here, to run repeatedly:
  XELNetworkMaster::run();
}