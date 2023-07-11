// Copyright (c) 2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SHIKAKU__NODE_DATA_PROCESSOR_HPP_
#define SHIKAKU__NODE_DATA_PROCESSOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"

namespace shikaku
{

class DataProcessor
{
public:
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
  explicit DataProcessor(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node;
  sensor_msgs::msg::JointState joint_state_msg;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state;
  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;
  float degree2Rad(float degree);
  void registerJoint(const std::string & joint_name, const double & pos);
};

}  // namespace shikaku

#endif
