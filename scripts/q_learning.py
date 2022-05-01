#!/usr/bin/env python3

from ast import NameConstant
import rospy
import numpy as np
import os
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveObjectToTag, QMatrixRow

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"


def QMatrix_get(qmatrix, x, y):
    return qmatrix.q_matrix[x].q_matrix_row[y]

def QMatrix_set(qmatrix, x, y, val):
    qmatrix.q_matrix[x].q_matrix_row[y] = val
    return True
    

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
        print(self.action_matrix)
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
        print(self.actions)

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        print(self.states)


        # --------------- Our Code after this point --------------- #
        # Initialize QMatrix publisher
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # Initialize Robot Action Publisher
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # subscribe to the reward received from the reward node
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.q_learning_reward_recieved)

        # Initialize Training Variables
        self.converged = False # boolean to check if converged
        self.current_state = 0 # current state
        self.convergence_threshold = 0.001 # Similarity between two vals to be considered "the same"
        self.convergence_max = 1000 # max number of iterations without change before convergence
        self.convergence_counter = 0 # counter for convergence

        # Initialize Q Matrix of size 64 (states) x 9 (actions) and publish it
        self.n_actions = 9; self.n_states = 64
        self.q_matrix = self.initialize_q_matrix(self.n_states, self.n_actions)
        self.matrix_pub.publish(self.q_matrix)
        
        
        # Sleep before publishing first action to ensure that all subscribers are ready
        rospy.sleep(3)

        # Take and publish first random action
        rand_a, self.current_state = self.random_action()
        self.action_pub.publish(rand_a['object'], rand_a['tag'])

    def initialize_q_matrix(self, states, actions):
        new_matrix = []
        for i in range(states):
            empty_row = np.zeros(actions)
            new_matrix.append(QMatrixRow(empty_row))
        q_matrix = QMatrix(q_matrix=new_matrix)
        #empty_matrix = np.zeros((states, actions))
        #q_matrix = QMatrix(q_matrix=QMatrixRow(empty_matrix))
        return q_matrix

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

    # random_action uses the current state and the action matrices to return a random action
    # that can be taken from the current state, and the new state that the action leads to.
    def random_action(self):
        # TODO: Pick a random action given current state and action matrix
        possible_end_states = []
        for i in range(self.n_states):
          if self.action_matrix[self.current_state][i] != -1:
            possible_end_states.append(i)
        if (len(possible_end_states) == 0): # if no possible actions, return None
          return None, None
        end_state = np.random.choice(possible_end_states)#np.random.randint(low=0, high=(len(possible_actions) - 1))
        return self.actions[int(self.action_matrix[self.current_state][end_state])], end_state


    def check_converged(self, new_q_matrix):
      for i in range(self.n_states):
        for j in range(self.n_actions):
          if (abs(QMatrix_get(self.q_matrix, i, j) - QMatrix_get(new_q_matrix,i,j)) > self.convergence_threshold):
            self.convergence_counter = 0
            return False
      self.convergence_counter += 1
      if (self.convergence_counter < self.convergence_max):
        return False
      return True

    # Function is called when a reward is received from the reward node
    def q_learning_reward_recieved(self, reward_msg):
      print("Recieved a reward message")
      print("i: ", reward_msg.iteration_num, "val: ", reward_msg.reward)

      if (reward_msg.iteration_num % 100  == 0):
          print(self.q_matrix)

      # TODO Replace this with actual code for updating QMatrix based on reward and previous action
      new_q_matrix = self.q_matrix
      if (reward_msg.reward > 1):
          QMatrix_set(new_q_matrix,0,0,reward_msg.reward)
          
      self.check_converged(new_q_matrix)
      self.q_matrix = new_q_matrix # After checking convergence, update q_matrix
      print(QMatrix_get(self.q_matrix, 0, 0))
      if self.converged:
        self.save_q_matrix()
      else:
        print("Not yet converged")

        rand_a, possible_state = self.random_action()
        if rand_a is not None:
          self.current_state = possible_state
          self.action_pub.publish(rand_a['object'], rand_a['tag'])
        else:
          print("No possible actions")
          
          # Reset simulation
          self.current_state = 0

          # Publish random action
          rand_a, self.current_state = self.random_action()
          self.action_pub.publish(rand_a['object'], rand_a['tag'])
      return

if __name__ == "__main__":
    node = QLearning()

    rospy.spin()
