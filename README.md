# q_learning_project

**Team Members**: Nick Auen, Rory Butler

## Implementation Plan

### Q-learning algorithm
- Executing the Q-learning algorithm
    - **Plan**: First, set up the required matrices. Then, use the phantom robot to learn how to use the `/q_learning/robot_action` topic. From there, use the phantom robot to implement Q-learning and finally transfer to the virtual reset world.
    - **Testing**: Run the virtual reset world several times to make sure the Q-matrix updates and begins to converge on reasonable values.
- Determining when the Q-matrix has converged
    - **Plan**: Observe a few runs of the training method and watch how quickly the Q-matrix changes. Hand-craft a threshold that both ensures the training does not run too long, but also produces a mostly unchanging Q-matrix.
    - **Testing**: Run the algorithm a few times and make sure the Q-matrix converges to a similar matrix each time.
- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
    - **Plan**: At each state in the process, find the state in the Q-matrix, then take the action with the highest Q-value.
    - **Testing**: Run the algorithm with full exploitation and make sure it makes no mistakes.

### Robot perception
- Determining the identities and locations of the three colored objects
    - **Plan**: Reuse the code from the line following lab to find the desired color in the camera feed. Since the objects are vertical, it may require changing where in the image the color is searched for. Finally, use the lidar sensor to determine the distance.
    - **Testing**: Point the robot at the colored objects in various positions, and have the robot print when and where it detects them.
- Determining the identities and locations of the three AR tags
    - **Plan**: Use the `aruco.detectMarkers` function to determine the left-right position of the marker in front of the robot, and the lidar sensor to determine the distance.
    - **Testing**: Point the robot at the tags in various positions, and have the robot print when and where it detects them.

### Robot manipulation & movement
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm
    - **Plan**: Experiment with the arm controls to find a decent pose for picking up the tube. Then, experiment with grasping strength to make sure the tube is not bent, and find a good location to hold the obect without it obscuring the camera.
    - **Testing**: Test the arm on the object directly in front of the robot, then in several positions around the robot.
- Navigating to the appropriate locations to pick up and put down the colored objects
    - **Plan**: If the current object/location is not currently in view, the robot can spin in circles until it detects the object. Then, use proportional control to rotate to and move towards the colored objects without knocking them over, and similarly to approach the fiducials without bumping into them.
    - **Testing**: Place the robot in an area with the colored objects and move them around, making sure the robot correctly follows it.

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
### High-level description
### Q-learning algorithm description
### Robot perception description
### Robot manipulation and movement
### Challenges
### Future work
### Takeaways
