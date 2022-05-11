# q_learning_project
Sam Shatzkin & Josh Garza

Gif 1 - Perception and Movement
![q_learning_1.gif](https://github.com/Sshatzkin/warmup_project/blob/main/q_learning_1.gif)

Robot recieves actions, goes to correct baton, waits while it would pick up baton, then drives to correct AR tag, and waits while it would put the AR tag down. After simulating placing the baton, it sends a confirmation of completion, and begins its next task. 


## Implementation Plan

- __Q-learning algorithm__:
  - __Executing the Q-learning algorithm__: We will implement the Q-Learning algorithm by implementing the steps described in class, by traversing a variety of paths through the space in order to optimize the Q-matrix. This step can be tested by reviewing the Q-matrix at each timestep to ensure that it is being updated as would be expected.
  - __Determining when the Q-matrix has converged__: We will know that the Q-matrix has converged once it remains static after a certian number of runs. Testing will be required to determine how many runs is reasonable to execute to ensure that our Q-matrix has converged.
  - __Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward__
  If the Q-matrix has converged, we should be able to follow a path through the Q-matrix by taking the maximum reward action at each timestep. We can test this by visually inspecting the Q-matrix and ensuring our calculated path follows the expected route.
- __Robot perception__:
  - __Determining the identities and locations of the three colored objects__: We can use lidar data to perform our basic object detection needs, and we can sample pixel data from the forward-facing cameras to determine what color each of the objects is.
  - __Determining the identities and locations of the three AR tags__: The front-facing camera can capture images that we forward to the aruco library which will help us to identify each tag. We can easily check if the robot is correctly identifying tags by checking it against each of the tags.
- __Robot manipulation & movement__:
  - __Picking up and putting down the colored objects with the OpenMANIPULATOR arm__: We will use scan and camera data to line the robot up with the object that we want to pick up, and then we will execute a hard-coded sequence for maneuvering the arm into the right position, grasping the rod, and lifting it up out of the way of sensors.
  - __Navigating to the appropriate locations to pick up and put down the colored objects__: This can be achieved using methods similarly to the person-following behavior from the warmup project. Once we've used our Q-Learning algorithm to decide what our next target state is, we can use LIDAR and camera to identify the target location, and a PID controller to approach it.

### Timeline

- __Q-learning algorithm__ - We expect to complete this by the first Q-Matrix deadline on Tuesday, May 3 at 11:00am
- __Robot perception__ - Friday May 6
- __Robot manipulation & movement__ - Monday May 9

## Objectives

The goal of this project is to use Q-learning and robot perception capabilities to allow a robot to arrange a set of colored batons in front of their matching AR tag. We used the Q-learning algorithm to discover the strategy which will provide maximum reward, and then we will use the robot's perceptive abilities to identify the items involved in each action, and use the robot's arm to physically move the baton to its correct spot.

## High Level Description

Before the robot actually executes any action, we perform a training loop based on the Q-learning algorithm, which will allow us to discover a path from the initial state of all colored batons in their starting positions, to a final state in which each baton is in front of the correct AR tag.

In order to achieve this end goal, we must first build our Q-matrix, and fill it out by simulating the pursuit of many possible paths from the initial state to the various possible end states, and updating the Q-matrix according to the rewards recieved. We repeatedly simulate a random action from the set of currently viable actions until an end-state with all 3 batons placed is reached, then we reset the simulation and begin again. This loop will continue on forever until the Q-matrix has been unchanged for a set number of iterations, and then we assume that it has converged and we have completed filling out our Q-matrix.

Once we have our finalized Q-matrix, we can follow the path of the maximum reward action at each state from the intial state to reach the maximum reward end state, which will be the correct position of each of the batons.

## Q-Learning Algorithm Description

### Selecting and Executing Actions

We have defined a function ``random_action()``: Uses the current state and the action matrices to return a random, valid action that can be taken from the current state, and the new state that results from the action. We perform a loop in which we execute a random valid action, and then wait for a reward response, before executing the next action. When we recieve a reward, we call ``q_learning_reward_recieved()``, in which we update our Q-Matrix and then take another ``random_action()``. If no valid actions are available, we reset to the initial state and begin again.

### Updating the Q-Matrix

``update_q_matrix()``: Implements the update step of the Q-Learning algorithm, using an alpha of 1 and a gamma of 0.9 to update the q_matrix.
``q_learning_reward_recieved()``: Calls ``update_q_matrix()`` and supplies the reward value to that function.

### Determining Convergence

``update_q_matrix()`` returns a boolean to indicate whether the Q-matrix was actually changed during the update phase. ``check_converged()``: Keeps track of how many actions have been taken since the last update was made to the matrix, and if it has surpassed the maximum number of required actions, it returns true. Convergence is checked by calling this function each time a reward is published by the reward node, called by ``q_learning_reward_recieved()``.

### Execution

We have written a python script, ``action_publisher.py``, which loads the trained q_matrix, and publishes the optimal action at each state to ``/q_learning/robot_action``. The ``robot_controller`` node listens for this action, performs it, and sends a confirmation boolean message on ``/q_learning/action_conf``. When the action_publisher recieves this confirmation message, it publishes the next optimal action.

## Robot Perception Description

### Identifying the locations and identities of each of the colored objects

The ``robot_controller`` node keeps track of its current target, and target type, either a specific baton, a specific ar tag, or nothing. When the robot is currently targetting a baton, it uses data from the camera and LIDAR scanner to find its target. There are 3 key functions that help this happen.

``image_callback(self, msg)``: Each time that the camera returns an image, the robot_controller uses that image to set it's own __horizontal error__, the horizontal distance from it's current target from the center of the image. To find a specific baton, we use a method similar to what was used in the line-following project, to get the location of our current target baton in the image, and steer toward it.

``scan_callback(self, data)``: This function is called each time the scanner returns data, and it is uses the scanner measurements from 359 0, and 1 degrees to set the __distance error__, a value that is used to stop the robot a certain distance from a target object.

``update_movement(self)``: This function uses the __horizontal error__ and __distance error__ to set the speed and rotation of the robot, steering it toward the target object.

### Identifying the locations and identities of each of the AR tags

Using the ARUCO open cv library, we were able to extend the abilities of the ``image_callback`` function to identify AR tags in the captured images.

The ARUCO library provides a function that gives the locations of each of the four corners of the AR tag, and we wrote a custom function, ``tag_center``, which calculates the center of an AR tag given the four corners of the tag. This distance between this calculated AR tag center and the center of the image is used to calcluate the __horizontal error__.

## Robot Manipulation and Movement

### Moving to the right spot in order to pick up a colored object

As described above, the ``image_callback`` and ``scan_callback`` functions set the horizontal and distance error values, which are used by the ``update_movement`` function to set forward and rotation rates.

If the target object is not in view, the robot rotates until the target is identifited.

If the target is in view, the robot rotates toward it until it is centered.

When the target object is within a certain range of the center of the screen, the robot approaches it slowly until it comes within grasping range of the target object.

### Picking up the colored object

TODO: Not yet implemented

### Moving to the desired destination (AR tag) with the colored object

This step primarily uses the same three functions as moving to pick up the baton. ``image_callback`` and ``scan_callback`` set the horizontal and distance error values to the AR tag, and the robot uses a proportional control mechanism to steer directly toward the tracked object.

### Putting the colored object back down at the desired destination

TODO: Not yet implemented

## Challenges

Describe the challenges you faced and how you overcame them.
The first challenge that we faced was deciding on the structure of the entire codebase - how should the components be structured and how should they communicate with each other. For the high-level structure, we opted for our ``action_publisher`` and ``robot_controller`` set up, where the ``action_publisher`` interacts with the q-matrix to figure out what high-level actions need to be taken, it sends them to the ``robot_controller``, and awaits a confirmation before proceeding. The ``robot controller`` handles all of the lower-level perception and movement requirements of the robot.

We had to make these kinds of architecture decisions again within ``robot_controller``, and opted for a configuration in which the ``camera_callback`` and ``scan_callback`` functions update global error variables, which are used by a seperate movement function to determine how the robot should behave.

TODO: Arm-related challenges

## Future Work

Future work (1 paragraph): If you had more time, how would you improve your implementation?
The color-only method for determining horizontal error is pretty limited, and a combination of image analysis and a more complex scanner-based object detection could be much more effective for detecting the batons.

TODO: Arm-related future work

## Takeaways

- Takeaway 1 - A project like this one that has one large python file which contains most of the work for this project can be very difficult to divide up among teammates. Many of the steps of this project felt sequential, like it's hard to develop each one without its precoursor, which made it difficult for us to work simultaneously. It could have been very valuable for us to figure out how certain components could have been developed and tested seperately from the whole.
- Takeaway 2 - TODO

## How to Use

You need to open 5 terminals when executing this on a physical robot.

1. ``roscore``
2. ssh into robot and run ``bringup``
3. ssh into robot and run ``bringup_cam``
4. run ``rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw``
5. run ``roslaunch q_learning_project action.launch``