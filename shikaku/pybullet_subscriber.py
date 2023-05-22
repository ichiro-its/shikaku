#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
import logging

class PybulletSubscriber(Node):
    def __init__(self):
        super().__init__('pybullet_subscriber')

        self.get_logger().info("Subscribing to joint states...")

        self.subscripton = self.create_subscription(JointState, 'joint_states', self.updateJointData, 10)
        self.jointAngles = [0.0] * 20
        self.jointID = [0] * 20
        self.jointNames = []
        self.initPybullet()

    def initPybullet(self):
        self.get_logger().info("Initializing simulation...")

        # Initialize pybullet simulator
        cid = p.connect(p.SHARED_MEMORY)
        if(cid < 0):
            p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        # Load URDFs
        startPosition = [0,0,2]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.fieldID = p.loadURDF("/urdf/field.urdf")
        self.robotID = p.loadURDF("/urdf/robot.urdf", startPosition, startOrientation)
        

        # Get joint information in URDF
        self.numJoints = p.getNumJoints(self.robotID)
        for joint in range(self.numJoints):
            self.jointNames.append(p.getJointInfo(self.robotID, joint)[1].decode('UTF-8'))       

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
                p.setJointMotorControl2(self.robotID, self.jointID[i], p.POSITION_CONTROL, targetPosition=self.jointAngles[i])
            
            p.stepSimulation()
            time.sleep(0.01)
        except:
            logging.exception("Error updating joint data")
            pass

def runPybulletVisualization(args=None):
    rclpy.init(args=args)

    try:
        pybullet_subscriber = PybulletSubscriber()
        rclpy.spin(pybullet_subscriber)
    except rclpy.exceptions.ROSInterruptException:
        pass

    pybullet_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    runPybulletVisualization()
