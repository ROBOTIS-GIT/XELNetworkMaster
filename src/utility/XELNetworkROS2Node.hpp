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

#ifndef XEL_NETWORK_ROS2_NODE_H_
#define XEL_NETWORK_ROS2_NODE_H_

#include "XELNetworkCommon.h"

typedef struct TopicInfo{
  TopicItemHeader_t header;
  ros2::CallbackFunc callback_func;
  uint8_t xel_id;
}TopicInfo_t;

class XELNetworkROS2Node:public ros2::Node
{
  public:
    XELNetworkROS2Node()
    : Node("xelnetwork_comm")
    {}

    template <typename MsgT>
    bool createNewTopic(TopicInfo_t& info)
    {
      bool ret = false;
      ros2::Publisher<MsgT>* p_pub;
      ros2::Subscriber<MsgT>* p_sub;

      switch(info.header.mode)
      {
        case TopicMode::PUB:
          p_pub = this->createPublisher<MsgT>((const char*)info.header.name);
          if(p_pub != nullptr){
            this->createWallTimer(info.header.pub_interval_ms, info.callback_func, (void*)&info, p_pub);
            ret = true;
          }
          break;

        case TopicMode::SUB:
          p_sub = this->createSubscriber<MsgT>((const char*)info.header.name, info.callback_func, (void*)&info);
          if(p_sub != nullptr){
            ret = true;
          }
          break;
      }

      return ret;
    }

    void deleteTopic(TopicInfo_t& info)
    {
      switch(info.header.mode)
      {
        case TopicMode::PUB:
          this->deletePublisher((const char*)info.header.name); break;

        case TopicMode::SUB:
          this->deleteSubscriber((const char*)info.header.name); break;
      }
    }

  private:
};

#endif /* XEL_NETWORK_ROS2_NODE_H_ */