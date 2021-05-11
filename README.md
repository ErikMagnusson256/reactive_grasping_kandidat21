
This code can control an UR10 robot arm and an RG2-FT gripper to execute simple reactive grasping tasks.
Developed for a bachelor thesis at Chalmers University of Technology during spring 2021. EENX15-21-39

This code will not be supported  in any way, shape or form in the future and is left as is!!!

Runs on Ubuntu 20.04
ROS noetic

This code uses the Universal Robot Driver to control the UR10 robot arm and an interface to control the RG2-FT through modbus has been created. It is far from ideal but works for the task at hand.

Program is currently set to execute a preplanned set of grasping executions that were relevant to test our algorithms. This is programed into mainController.py

The RG2-FT Computebox and UR10 robot arm is connected to the same network.
ROScore is started through the ur_robot_driver bringup launch file.

To run this code:
In terminal 1: cd reactive_grasping_kandidat21/catkin_kandidat21
		source devel/setup.bash
		roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=ENTER_ROBOT_IP_HERE
		
In terminal : cd reactive_grasping_kandidat21/catkin_kandidat21
		source devel/setup.bash
		rosrun reactive_grasping_pkg gripperInterface.py
		
In terminal 1: cd reactive_grasping_kandidat21/catkin_kandidat21
		source devel/setup.bash
		rosrun reactive_grasping_pkg mainController.py
