# Firebird HW4 - Fall 2024

Algorithm Description:

The approach that we chose to take was a modified right wall following search. We implemented this inside of the timer callback function that the robot controller called every 0.25s. The robot reads the lidar scan data. The first check performed after reading the scan data is obstacle detection. If the robot finds an obstacle in front of it within its safe stop distance, then it stops forward motion and rotates to its left.  Only after this check does the robot proceed to its wall following logic. 

The wall follow logic is implemented as follows: Check if the robot is within the specified distance to a wall on its right. If so, turn left – away from the wall. Otherwise, check if the robot is within a larger distance to a wall on its right. If so, turn right, towards the wall. Finally, if the robot does not detect a wall to its right or an obstacle in front, it will choose a random angular velocity within a reasonable range and go forward while turning, mimicking a random zig zag pattern to find a new wall to follow.

To summarize, the robot stops and turns left if an obstacle is found. If the robot is too close to a wall on the right, it turns away from the wall. If the robot is too far from a wall on the right, it turns right. Otherwise, it goes forward and left and right searching for a new wall to follow.

We chose to tune this controller so that it lost the wall every once in a while. This was both to avoid obstacles including the table in the lab and to try to get tags located away from walls in the middle of the room. 


# f24_robotics_HW4
