 * Title: Contact Sensor Based Implementation of iRobot Create Navigation Around Obstacles to Goal Postions
 * Author: Dennis Melamed
 * 
 * Based off of the pathfinding algorithm know as "Bug 2." 
 * A more complete explanation with visuals (by Howie Choset at Carnegie Mellon University) can be found here: 
 * http://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
 * 
 * 
 * Overall Process:
 * -Calculate shortest distance to target, record the angle from the target to a zero angle position (this describes the line to the target, aka the "m-line")
 * -Drive along the m-line until the target is reached or a collision is detected
 * -If a collision is detected, follow the edge of the object collided against clockwise until the m-line is regained
 * -Continue driving towards target following the m-line, repeat object circumnavigation as required until target reached.
 * 
 * More Detailed Flow:
 * -An angle to the goal position is found and rotated to
 * -The robot moves forward until a collision is detected or the goal position is reached
 * -if a collision is detected:
 * 		-an angle respresenting the line from the current position to the goal position is stored
 * 		-the robot drives back a short distance, 
 * 		-rotates 90 degrees counter-clockwise,
 * 		-drives forward a robot-length,
 * 		-rotates 90 degrees clockwise,
 * 		-drives forward checking for a collision
 * 		-if a collision is detected:
 * 			- the above process repeats itself until a collision is not detected
 * 		-if there is no collision after a certain distance:
 * 			- this indicates the robot has gone off an edge of the obstacle
 * 			-The robot drives forward an additional robot-length,
 * 			-rotates 90 degrees clockwise
 * 			-drives forward a robot-lenght
 * 			-begins testing again as though a collision had just been detected
 * 		-Once the robot is at the same angle from the origin as the goal position is, the m-line has been regained
 * 		-The robot rotates to face the goal position and begins driving toward it again
