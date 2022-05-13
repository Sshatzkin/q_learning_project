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
import moveit_commander


def convert_hsv (h, s, v):
    new_h = h/2 - 1
    new_s = s * 256 - 1
    new_v = v * 256 - 1
    return np.array([new_h, new_s, new_v])

def tag_center(tag):
    ave_x = 0
    ave_y = 0
    for i in range(4):
        ave_x += tag[i][0]
        ave_y += tag[i][1]

    return [ave_x/4, ave_y/4]

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


        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # Subscribe to LIDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
       

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_recieved)

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

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

        # Error vals used for proportional control, set by perception functions
        self.horizontal_error = 0

        self.distance_error = 0

        # Used to keep track of whether target item is in camera view
        self.target_in_view = False

        # Counter used to ensure robot does not pick up colored
        #   item until it is definitely in front of it
        self.arrived_at_target_counter = 0

        # True when robot is holding an item
        self.holding_item = False

        rospy.sleep(3)

    def image_callback(self, msg):
        #print("See image")
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        h, w, d = image.shape
        #cv2.imshow("window", image)
                
        if (self.current_target_type == "baton"):
            #print("Case 1")
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
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
            #print("Case 2")
            grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            # corners is a 4D array of shape (n, 1, 4, 2), where n is the number of tags detected
            # each entry is a set of four (x, y) pixel coordinates corresponding to the
            # location of a tag's corners in the image

            # ids is a 2D array array of shape (n, 1)
            # each entry is the id of a detected tag in the same order as in corners

            # rejected_points contains points from detected tags that don't have codes matching the dictionary
            
            #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            
            if (len(corners) > 0):
                #centers = []
                for i, tag in enumerate(corners):
                    #print("Tag ",ids[i][0], ": ", tag[0])
                    if (ids[i] == self.current_target):
                        #print("match tag")
                        center = tag_center(tag[0])
                        self.horizontal_error = (center[0] - w/2) / (w /2)
                        self.target_in_view = True
                        break
                    else:
                        self.target_in_view = False
                        self.horizontal_error = 0
                
            else:
                self.target_in_view = False
                self.horizontal_error = 10
        else:
            #print("Case 3")
            self.horizontal_error = 0
    
        #cv2.imshow("window", image)
        #cv2.waitKey(3)

        self.update_movement()

    def scan_callback(self, data):
        center_average = 0
        angles = [359, 0, 1]
        for i in angles:
            #print(i, ": ", data.ranges[i])
            if data.ranges[i] == 0.0:
                center_average += 0.01
            if data.ranges[i] > 4:
                center_average += 0.01
            else:
                center_average += data.ranges[i]
        center_average = center_average / len(angles)
        if center_average != 0.0 and center_average > 0.4:
            self.distance_error = center_average / 3.5
        else:
            self.distance_error = 0
        self.update_movement()
        return


    def update_movement(self):
        #print("Horizontal Error: ", self.horizontal_error)
        #print("Distance Error: ", self.distance_error)
        if self.current_target_type != None: # and self.arrived_at_target_counter <= 120:
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
                forward_speed = (self.distance_error + 0.021) * 0.5
                self.twist.linear.x = forward_speed

            # If centered and close, increment distance
            if (self.horizontal_error < 0.1 and self.distance_error == 0):
                self.arrived_at_target_counter += 1
            else:
                self.arrived_at_target_counter = max(0, self.arrived_at_target_counter - 1)
            
            print("Counter: ",self.arrived_at_target_counter)

            if (self.arrived_at_target_counter > 100 and self.holding_item == False and self.current_target_type == 'baton'):    
                self.holding_item = True
                #if self.current_target_type == 'baton':
                self.pick_up_baton()
            elif (self.arrived_at_target_counter > 100 and self.holding_item == True and self.current_target_type == 'artag'):
                self.holding_item = False
                #elif self.current_target_type == 'artag':
                self.place_baton()

                    
        else:
            self.twist.angular.z = 0
            self.twist.linear.x = 0

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
        # rospy.sleep(3)
        # TODO: Implement pickup
        # Move arm back
        arm_goal = [0.0, np.radians(-93), np.radians(62), np.radians(33)]
        self.move_group_arm.go(arm_goal, wait=True)
        self.move_group_arm.stop()

        # Open gripper
        gripper_goal = [0.019, -0.019]
        self.move_group_gripper.go(gripper_goal, wait=True)
        self.move_group_gripper.stop()

        # Stretch arm out to grab 
        arm_goal = [0.0, np.radians(38), np.radians(-3), np.radians(-29)]
        self.move_group_arm.go(arm_goal, wait=True)
        self.move_group_arm.stop()
        # Move forward to avoid bumping dumbbell in gazebo
        rospy.sleep(1)

        # Grab dumbbell
        gripper_goal = [-0.008, 0.008]
        self.move_group_gripper.go(gripper_goal, wait=True)
        self.move_group_gripper.stop()

        # Lift up
        arm_goal = [0.0, 0.0, 0.0, 0.0]
        self.move_group_arm.go(arm_goal, wait=True)
        self.move_group_arm.stop()

        print("Current Action: ", self.current_action)
        self.current_target_type = "artag"
        self.current_target = self.current_action.tag_id
        self.target_in_view = False
        self.arrived_at_target_counter = 0
        print("Current target updated to tag: ", self.current_target)
        return

    def place_baton(self):
        print("Placing Baton")
        rospy.sleep(3)
        # TODO : Implement placement
        arm_goal = [0.0, np.radians(38), np.radians(-3), np.radians(-29)]
        self.move_group_arm.go(arm_goal, wait=True)
        self.move_group_arm.stop()
        
        gripper_goal = [0.019, -0.019]
        self.move_group_gripper.go(gripper_goal, wait=True)
        self.move_group_gripper.stop()

        arm_goal = [0.0, 0.0, 0.0, 0.0]
        self.move_group_arm.go(arm_goal, wait=True)
        self.move_group_arm.stop()

        gripper_goal = [0, 0]
        self.move_group_gripper.go(gripper_goal, wait=True)
        self.move_group_gripper.stop()

        self.holding_item = False
        self.arrived_at_target_counter = 0
        self.target_in_view = False
        self.current_target = None
        self.current_target_type = None
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