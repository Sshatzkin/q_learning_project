#!/usr/bin/env python3

from ast import NameConstant
import rospy
import numpy as np
import os
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

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


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))


        # --------------- Our Code after this point --------------- #
        # Publish Q Matrix
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # Publish Robot Action
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.q_learning_reward_recieved)

        self.converged = False

        # Initialize Q Matrix of size 64 (states) x 9 (actions) and publish it
        n_actions = 9; n_states = 64
        self.q_matrix = self.initialize_q_matrix(n_states, n_actions)
        self.matrix_pub.publish(self.q_matrix)

        # Publish first random action
        rand_a = self.random_action()
        self.action_pub.publish(rand_a)

    def initialize_q_matrix(self, states, actions):
        empty_matrix = np.zeros((states, actions))
        q_matrix = QMatrix(q_matrix=QMatrixRow(empty_matrix))
        return q_matrix

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

    def random_action(self):
        # TODO: Pick a random action given current state and action matrix
        # TODO: No clue if I coded this right, have to check format of msg
        i = np.random.randint(low=0, high=(len(self.actions) - 1))
        return self.actions[i]

    def q_learning_reward_recieved(self, reward_msg):
      
      print(reward_msg)

      # TODO Update Q Matrix based on Reward, and publish new matrix

      if self.converged:
        self.save_q_matrix()
      else:
        print("Not yet converged")

        # TODO If actions available, take random action and publish

        # TODO Else, reset simulation and take and publish random action

      return

if __name__ == "__main__":
    node = QLearning()
