'''
The program will use the Simple Commander API to localize the robot in the init_position.
The program will send a first goal to the robot using the NavigateToPose action.
The first goal will be located inside the loading_position (see image below).
Once the robot is at the loading_position it has to get underneath the shelf and activate the elevator to carry it.

Notes:

o move the robot underneath the shelf and lift it you will not be able to use navigation
 (since it will detect the shelf as an obstacle). 
 For this, you can recycle the code you created for Checkpoint 9, for both: the simulation and the real robot.

Remember that when it has loaded the shelf, the shape of the robot will be larger
 (because now the robot is carrying the shelf). So you need to change its shape (robot footprint) in Nav2 in real time (while navigation is still running) to prevent the robot planning over places where the robot+shelf cannot fit.

'''