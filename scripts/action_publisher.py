#!/usr/bin/env python3

from ast import NameConstant
from turtle import update
import rospy
import numpy as np
import os
from std_msgs.msg import Bool
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
save_path = os.path.dirname(__file__) + "/q_matrix/"

class ActionPublisher(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("action_publisher")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
    
        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch pre-trained Q-matrix.
        # If the matrix doesn't exist, throw an error
        if os.path.exists(save_path + "q_matrix.txt"):
            print("AP: Existing q_matrix, loading...")
            self.q_matrix = np.loadtxt(save_path + "q_matrix.txt")

        else:
            print("AP: Pre-trained Q-matrix doesn't exist at the specified path.")
        
        # Initialize Robot Action Publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        rospy.Subscriber("/q_learning/action_conf", Bool, self.action_confirmed)

        # Initialize Training Variables
        self.completed = False # Keep
        self.current_state = 0 # current state
        self.last_state = None # last state
        self.last_action_i = None # last action

        self.n_actions = 9; self.n_states = 64
        
        # Sleep before publishing first action to ensure that all subscribers are ready
        rospy.sleep(3)

        # Take and publish first action
        self.last_state = self.current_state
        action, self.last_action_i, self.current_state = self.select_action()
        self.action_pub.publish(action['object'], action['tag'])


    def select_action(self):
        max_val = 0
        max_index = 0
        for i, val in enumerate(self.q_matrix[self.current_state]):
            if val > max_val:
                max_val = val
                max_index = i
        if max_val == 0:
            return None, None, None
        end_state = np.where(self.action_matrix[self.current_state] == max_index)[0][0]#self.action_matrix[self.current_state][max_index]
        print("Next Action: ", self.actions[max_index]) 
        print("Action Index: ", max_index)
        print("New State: ", end_state)
        print("______")
        
        return self.actions[max_index], max_index, end_state

    #def find_end_state (current_state, action):
    #    return np.where(action == self.)

    def action_confirmed(self, conf):
        print("AP: Recieved action confirmation: ", conf)
        rospy.sleep(0.5)
        
        self.last_state = self.current_state
        action, self.last_action_i, self.current_state = self.select_action()
        if action != None:
            self.action_pub.publish(action['object'], action['tag'])


if __name__ == "__main__":
    node = ActionPublisher()
    
    rospy.spin()
