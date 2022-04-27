# q_learning_project

Sam Shatzkin & Josh Garza

## Implementation Plan

A 1-2 sentence description of how your team plans to implement each of the following components of this project as well as a 1-2 sentence description of how you will test each component:

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