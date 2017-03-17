README.txt
---------------------------------------------

This submission includes two files:

myRobot.py
	- This file is the myRobot class that Houston built, with my implementation of constructA and getT.

robot_pub_sub.py
	- This file is the robot publisher and joint state subscriber. 
	- To run this file:
		This function requires the name of the file defining the dh parameters and joints for the robot to be passed as an argument.
		This json file name is passed in as a command line argument (-b) to this script as follows:
			rosrun robot_pub_sub.py -b <bot_file.json>

---------------------------------------------

Process for running code:
	Due to the handling of subscribing to the joint state, the code should be run in the following order to ensure the vizualization is happening:
		1. launch rviz
		2. launch joint state publisher
		3. launch robot_pub_sub.py as described above
	If the joint state publisher is not being run, then nothing will be visualized because the robot publisher will be waiting for data from the listener's callback before it will publish the joint parameters.
