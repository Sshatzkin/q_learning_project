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
  - __Determining the identities and locations of the three colored objects__:
  - __Determining the identities and locations of the three AR tags__:
- __Robot manipulation & movement__:
  - __Picking up and putting down the colored objects with the OpenMANIPULATOR arm__:
  - __Navigating to the appropriate locations to pick up and put down the colored objects__:

### Timeline

- __Q-learning algorithm__ - We expect to complete this by the first Q-Matrix deadline on Tuesday, May 3 at 11:00am
- __Robot perception__ - Friday May 6
- __Robot manipulation & movement__ - Monday May 9