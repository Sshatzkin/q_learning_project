#!/usr/bin/env python3

from ast import NameConstant
from turtle import color, update
import rospy
import numpy as np
import os
import cv2, cv_bridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow


def convert_hsv (h, s, v):
    new_h = h/2 - 1
    new_s = s * 256 - 1
    new_v = v * 256 - 1
    return np.array([new_h, new_s, new_v])

# Green, Blue, Pink
lower_colors = [convert_hsv(75, 0.40, 0.40), convert_hsv(180, 0.40, 0.40), convert_hsv(300, 0.40, 0.40)] 
upper_colors = [convert_hsv(80, 1, 1), convert_hsv(200, 1, 1), convert_hsv(330, 1, 1)]

color_index = {"green" : 0, "blue": 1, "pink": 2}

class RobotController(object):
    def __init__(self):
        rospy.init_node("robot_controller")

        # --- Initialize Vision ---
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)


        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_recieved)

        self.confirmation_pub = rospy.Publisher("/q_learning/action_conf", Bool, queue_size=10)

        # Current target can be "green", "blue", "pink", 1, 2, 3, or None if awaiting action
        self.current_target = None

        # Target type is "baton", "ar_tag" or None
        self.current_target_type = None

        self.current_action = None

        self.horizontal_error = 0

        rospy.sleep(3)

    def image_callback(self, msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
        if (self.current_target_type == "baton"):
            h, w, d = image.shape
            i = color_index[self.current_target]

            # Generate mask
            mask = cv2.inRange(hsv, lower_colors[i], upper_colors[i])

            M = cv2.moments(mask)

            # If any green pixels are found
            if M['m00'] > 0:
                # Center of the green pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # Visualize red circle in center
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

        cv2.imshow("window", image)
        cv2.waitKey(3)
    
        



    def action_recieved(self, action):
        print("RC: Recieved Action: ", action)
        self.current_action = action
        self.current_target = action.robot_object
        self.current_target_type = "baton"
        rospy.sleep(3)
        self.send_confirmation()
        return
        

    def send_confirmation(self):
        self.confirmation_pub.publish(True)
        print("RC: Sending Action Completion Confirmation")

        return

if __name__ == "__main__":
    node = RobotController()

    rospy.spin()