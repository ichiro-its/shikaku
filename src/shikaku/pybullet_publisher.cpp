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

#include "shikaku/pybullet_publisher.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

const char joint_id[20][22] = {
  "R_SHO_PITCH",  // 1
  "L_SHO_PITCH",  // 2
  "R_SHO_ROLL",   // 3
  "L_SHO_ROLL",   // 4
  "R_ELBOW",      // 5
  "L_ELBOW",      // 6
  "R_HIP_ROLL",   // 7
  "L_HIP_ROLL",   // 8
  "R_HIP_YAW",    // 9
  "L_HIP_YAW",    // 10
  "R_HIP_PITCH",  // 11
  "L_HIP_PITCH",  // 12
  "R_KNEE",       // 13
  "L_KNEE",       // 14
  "R_ANK_ROLL",   // 15
  "L_ANK_ROLL",   // 16
  "R_ANK_PITCH",  // 17
  "L_ANK_PITCH",  // 18
  "HEAD_PAN",     // 19
  "HEAD_TILT"     // 20
};

namespace shikaku
{

PybulletPublisher::PybulletPublisher(rclcpp::Node::SharedPtr node) : node(node)
{
  joint_state = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  joint_state_msg.header.stamp = node->now();

  for (int i = 0; i < 20; i++) {
    registerJoint(joint_id[i], 0.0);
  }

  current_joints_subscriber = node->create_subscription<CurrentJoints>(
    "/joint/current_joints", 10, [this](const CurrentJoints::SharedPtr message) {
      {
        for (const auto & joint : message->joints) {
          std::cout << "ID: " << joint.id << "POSITION: " << joint.position << "\n";
          joint_state_msg.position[joint.id - 1] += joint.position;
        }
      }
    });

  joint_state->publish(joint_state_msg);
}

void PybulletPublisher::registerJoint(const std::string & joint_name, const double & pos)
{
  joint_state_msg.name.push_back(joint_name);
  joint_state_msg.position.push_back(pos);
}

}  // namespace shikaku
