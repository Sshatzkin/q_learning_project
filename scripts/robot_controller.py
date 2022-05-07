#!/usr/bin/env python3

from ast import NameConstant
from turtle import update
import rospy
import numpy as np
import os
from std_msgs.msg import Bool
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow


class RobotController(object):
    def __init__(self):
        rospy.init_node("robot_controller")

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_recieved)

        self.confirmation_pub = rospy.Publisher("/q_learning/action_conf", Bool, queue_size=10)

        self.current_action = None

        rospy.sleep(3)

    def action_recieved(self, action):
        print("RC: Recieved Action: ", action)
        rospy.sleep(3)
        self.confirmation_pub.publish(True)
        print("RC: Sending Action Completion Confirmation")

if __name__ == "__main__":
    node = RobotController()

    rospy.spin()