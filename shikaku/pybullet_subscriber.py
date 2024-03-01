#!/usr/bin/env python3

# Copyright (c) 2021 ICHIRO ITS
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import logging
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class PybulletVisualizer(Node):
    def __init__(self):
        super().__init__('pybullet_visualizer')

        self.get_logger().info("Subscribing to joint states...")

        self.subscripton = self.create_subscription(
            JointState, 'joint_states', self.updateJointData, 10)
        self.jointAngles = [0.0] * 20
        self.jointID = [0] * 20
        self.jointNames = []
        self.initPybullet()

    def initPybullet(self):
        self.get_logger().info("Initializing simulation...")

        # Initialize pybullet simulator
        cid = p.connect(p.SHARED_MEMORY)
        if (cid < 0):
            p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(1)

        # Load URDFs
        startPosition = [0,0, 0.8]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.fieldID = p.loadURDF("/src/shikaku/urdf/field.urdf")
        self.robotID = p.loadURDF(
            "/src/shikaku/urdf/robot.urdf", startPosition, startOrientation)
        

        # Set array of Pybullet joint index
        self.JointPID = [0, 4, 2, 6, 3, 7, 9, 15, 8, 14, 10, 16, 11, 17, 12, 18, 13, 19, 20, 22]
        # Get joint information in URDF
        self.numJoints = p.getNumJoints(self.robotID)
        for joint in range(self.numJoints):
            # Set joint force to 1000
            p.setJointMotorControl2(self.robotID, joint, p.POSITION_CONTROL, targetPosition=0, force=1000)
            self.jointNames.append(p.getJointInfo(
                self.robotID, joint)[1].decode('UTF-8'))

    def updateJointData(self, msg):
        try:
            # Get joint data from publisher
            self.get_logger().info("Receiving message...")
            for name in msg.name:
                if name in self.jointNames:
                    JointID_msg = msg.name.index(name)
                    JointID_urdf = self.jointNames.index(name)
                    self.jointID[JointID_msg] = JointID_urdf
                    self.jointAngles[JointID_msg] = (msg.position[JointID_msg])

            # Visualize joint data in pybullet
            self.get_logger().info("Visualizing joint data...")
            for i in range(len(self.jointNames)):
                p.setJointMotorControl2(
                    self.robotID, self.JointPID[i], p.POSITION_CONTROL, targetPosition=self.jointAngles[i])

            p.stepSimulation()
            time.sleep(0.01)
        except:
            logging.exception("Error updating joint data")
            pass


def runPybulletVisualization(args=None):
    rclpy.init(args=args)

    try:
        node = PybulletVisualizer()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    runPybulletVisualization()
