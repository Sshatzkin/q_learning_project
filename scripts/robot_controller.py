#!/usr/bin/env python3

from ast import NameConstant
from turtle import color, forward, update
import rospy
import numpy as np
import os
import cv2, cv_bridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist, Vector3
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

        # Subscribe to LIDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
       

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_recieved)

        # --- Initialize Movement ---
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # -- Logic ---
        self.confirmation_pub = rospy.Publisher("/q_learning/action_conf", Bool, queue_size=10)

        # Current target can be "green", "blue", "pink", 1, 2, 3, or None if awaiting action
        self.current_target = None

        # Target type is "baton", "artag" or None
        self.current_target_type = None

        self.current_action = None

        self.horizontal_error = 0

        self.distance_error = 0

        self.target_in_view = True

        self.arrived_at_target_counter = 0

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
                self.target_in_view = True

                self.horizontal_error = (cx - w/2) / (w /2)

            else:
                self.target_in_view = False
                self.horizontal_error = 10

        elif (self.current_target_type == "artag"):
            self.horizontal_error = 0
        else:
            self.horizontal_error = 0
    
        cv2.imshow("window", image)
        cv2.waitKey(3)

        self.update_movement()

    def scan_callback(self, data):
        center_average = 0
        angles = [359, 0, 1]
        for i in angles:
            #print(i, ": ", data.ranges[i])
            if data.ranges[i] == 0.0:
                center_average += 0.01
            else:
                center_average += data.ranges[i]
        center_average = center_average / len(angles)
        if center_average != 0.0 and center_average > 0.2:
            self.distance_error = center_average / 3.5
        else:
            self.distance_error = 0
        self.update_movement()
        return


    def update_movement(self):
        print("Horizontal Error: ", self.horizontal_error)
        print("Distance Error: ", self.distance_error)
        if self.current_target_type != None:
            # If can see target (cam), turn toward it
            if (self.target_in_view):
                turn_speed = (- self.horizontal_error) * 0.4
                self.twist.angular.z = turn_speed
                self.twist.linear.x = 0
            else:
                self.twist.angular.z = 0.4
                self.twist.linear.x = 0

            # If target in center, approach
            if (self.target_in_view and self.horizontal_error < 0.1):
                forward_speed = self.distance_error * 0.5
                self.twist.linear.x = forward_speed

            if (self.horizontal_error < 0.1 and self.distance_error == 0 and self.current_target_type == "baton"):
                self.arrived_at_target_counter += 1
            else:
                self.arrived_at_target_counter = max(0, self.arrived_at_target_counter - 1)

            if (self.arrived_at_target_counter > 10):    
                self.pick_up_baton()

        self.twist_pub.publish(self.twist)
        return


    def action_recieved(self, action):
        print("RC: Recieved Action: ", action)
        self.current_action = action
        self.current_target = action.robot_object
        self.current_target_type = "baton"
        self.arrived_at_target_counter = 0
        #rospy.sleep(25)
        #self.send_confirmation()
        return

    def pick_up_baton(self):
        print("Picking up baton")
        rospy.sleep(5)
        self.current_target_type = "artag"
        self.current_target = self.current_action.tag_id
        self.target_in_view = False
        self.arrived_at_target_counter = 0
        print("Current target updated to tag: ", self.current_target)
        self.send_confirmation()
        return

        

    def send_confirmation(self):
        self.current_action = None
        self.current_target = None
        self.current_target_type = None
        self.confirmation_pub.publish(True)
        print("RC: Sending Action Completion Confirmation")

        return

if __name__ == "__main__":
    node = RobotController()

    rospy.spin()