#!/usr/bin/env python3

from ast import NameConstant
from turtle import update
import rospy
import numpy as np
import os
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
save_path = os.path.dirname(__file__) + "/q_matrix/"

class ActionPublisher(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("action_publisher")

        # Fetch pre-trained Q-Matrix 
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        print(self.action_matrix)
        


        # --------------- Our Code after this point --------------- #
        # Initialize Robot Action Publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.q_learning_reward_recieved)

        # Initialize Training Variables
        self.converged = False # boolean to check if converged
        self.current_state = 0 # current state
        self.last_state = None # last state
        self.last_action_i = None # last action
        self.convergence_threshold = 0.001 # Similarity between two vals to be considered "the same"
        self.convergence_max = 10000 # max number of iterations without change before convergence
        self.convergence_counter = 0 # counter for convergence

        self.alpha = 1 # learning rate
        self.gamma = 0.9 # discount factor

        # Initialize Q Matrix of size 64 (states) x 9 (actions) and publish it
        self.n_actions = 9; self.n_states = 64
        self.q_matrix = self.initialize_q_matrix(self.n_states, self.n_actions)
        self.matrix_pub.publish(self.q_matrix)
        
        
        # Sleep before publishing first action to ensure that all subscribers are ready
        rospy.sleep(3)

        # Take and publish first random action
        self.last_state = self.current_state
        rand_a, self.last_action_i, self.current_state = self.random_action()
        self.action_pub.publish(rand_a['object'], rand_a['tag'])

