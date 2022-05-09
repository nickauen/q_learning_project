# q_learning_project

**Team Members**: Nick Auen, Rory Butler

## Implementation Plan

### Q-learning algorithm
- Executing the Q-learning algorithm
    - **Plan**: First, set up the required matrices. Then, use the phantom robot to learn how to use the `/q_learning/robot_action` topic. From there, use the phantom robot to implement Q-learning and finally transfer to the virtual reset world.
    - **Testing**: Run the virtual reset world several times to make sure the Q-matrix updates and begins to converge on reasonable values.
- Determining when the Q-matrix has converged
    - **Plan**: Observe a few runs of the training method and watch how quickly the Q-matrix changes. Hand-craft a threshold that both ensures the training does not run too long, but also produces a mostly unchanging Q-matrix by the end of exploration.
    - **Testing**: Run the algorithm a few times and make sure the Q-matrix converges to a similar matrix each time.
- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
    - **Plan**: At each state in the process, find the state in the Q-matrix, then take the action with the highest Q-value.
    - **Testing**: Run the algorithm with full exploitation and make sure it makes no mistakes.

### Robot perception
- Determining the identities and locations of the three colored objects
    - **Plan**: Reuse the code from the line following lab to find the desired color in the camera feed. Since the objects are vertical, it may require changing where in the image the color is searched for (for example, a column in the center of the image). Finally, use the LiDAR sensor to determine the distance.
    - **Testing**: Point the robot at the colored objects in various positions, and have the robot print the identities of the objects and when and where it detects them.
- Determining the identities and locations of the three AR tags
    - **Plan**: Use the `aruco.detectMarkers` function to determine the left-right position of the marker in front of the robot, and the lidar sensor to determine the distance.
    - **Testing**: Point the robot at the tags in various positions, and have the robot print the identities of the objects and when and where it detects them.

### Robot manipulation & movement
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm
    - **Plan**: Experiment with the arm controls to find a decent pose for picking up the tube. Then, experiment with grasping strength to make sure the tube is not bent, and find a good location to hold the obect without it obscuring the camera.
    - **Testing**: Test the arm on the object directly in front of the robot, then in several positions around the robot.
- Navigating to the appropriate locations to pick up and put down the colored objects
    - **Plan**: If the current object/location is not currently in view, the robot can spin in circles until it detects the object. Then, use proportional control to rotate to and move towards the colored objects without knocking them over, and similarly to approach the fiducials without bumping into them.
    - **Testing**: Place the robot in an area with the colored objects and move them around, making sure the robot correctly follows it at a distance that allows for it to pick up the colored objects using the code previousy developed for picking up/putting down objects.

### Timeline

- May 2
    - Implement the Q-learning algorithm with the phantom robot

- May 4
    - Detect the colored object's locations
    - Detect the tag's locations

- May 5
    - Train the Q-learning algorithm in the virtual reset world
    - Program the robot to navigate to a destination with an object or tag

- May 7
    - Program the arm to pick up and put down an object

- May 8
    - Make sure everything works in a Gazebo world

- May 9
    - Run the physical robot through the process and capture recordings

## Writeup
### Objectives description
The goal of this project is to use a q-learning algorithm to enable our robot to learn how to organize three objects in front of three AR tags using reinforcement learning. Additionally, this project aims to use the robot's vision to recognize the differently colored objects and utilize the robot's "arm" to interact with these three objects and place them in the proper locations (in front of the right AR tags) after the q-learning phase. 
### High-level description
In order for the robot to learn which colored object belongs in front of which AR tag, the q-learning algorithm iterates through multiple matrices of actions and rewards in order to generate a converged q-matrix of actions that capture the optimal set of actions that the robot should take in order to reach the correct goal. To create this converged q-matrix, the algorithm randomly selects an action from a set of valid actions (actions that can currently be taken). After this selection, the algorithm receives a "reward" (100 = correct action, 0 = incorrect action) which is then incorporated into the q-matrix. At the end of convergence, the greatest values in the q-matrix indicate the optimal set of actions to reach the goal state.
### Q-learning algorithm description
- Selecting and executing actions for the robot (or phantom robot) to take
   - In order to ensure that a valid action is selected (value != -1), an action array is created with all valid actions for the current state (those actions which != -1). This occurs in the *for loops* within def run(self). A weighted random choice is then made from this generated array using the *choices* function. This selected action is then used to find the next state of the system (selecting *from action_matrix*). This chosen action is then published using *RobotMoveObjectToTag*.
- Updating the Q-matrix
   - The q-learning algorithm listens for the reward publisher which determines the reward amount based on the published action. If the reward is positive (100), the q_matrix is then updated using the q-learning algorithm (reward value + discount factor (0.8) * np.max). If this newly calculated value differs from the current value, the q_matrix csv file is resaved.  
- Determining when to stop iterating through the Q-learning algorithm
   - The Q-learning algorithm can stop iterating once the q_matrix converges. Convergence occurs when the values of the matrix cease to change (the optimal path has been identified and optimized for). If the convergence_count variable reaches 25 or greater, this means that the q-learning algorithm has gone through 25 iterations without any changes to the CSV file. This likely means that the q-matrix has converged and the optimal path has been identified.
- Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
   - This has yet to be done for this section of the project.
### Robot perception description
- Identifying the locations and identities of each of the colored objects
    - The colors of each of the tubes (green, pink, and blue) were identified using a narrow range of RGB values. To control for other possible objects with similar colors that the camera might pick up, we exclude those pixels in the bottom half of the image. The robot orients itself with the colored tubes directly in front of it by moving aligning itself to place the colored tube in the center pixel of the image.
- Identifying the locations and identities of each of the AR tags
    - The AR tags are identified using the ArUco library implemented through the *aruco* module in OpenCV. The dictionary *DICT_4X4_50* is used for this specific implementation. This library automatically identifies the corners of the tags and the IDs of them. Similar to identifying the locations of the colored tubes, the robot aligns itself so that the target AR tag is placed in the center pixel of the image. 
### Robot manipulation and movement
- Moving to the right spot in order to pick up a colored object
- Picking up the colored object
- Moving to the desired destination (AR tag) with the colored object
- Putting the colored object back down at the desired destination
### Challenges
- *One paragraph*
### Future work
- *One paragraph*
### Takeaways
- *First bulletpoint*
- *Second bulletpoint*
