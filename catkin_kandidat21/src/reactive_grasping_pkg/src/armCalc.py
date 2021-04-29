#!/usr/bin/env python
#TODO add description of file

import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg
import math
import numpy as np
import tf_conversions
from geometry_msgs.msg import Quaternion
from tf.transformations import *
import vectors #sudo pip3 install vectors



'''
Interface between our ROS network and the Universal Robot Driver which in turn controls the UR10 Robot arm
This interface enables easy functions to control the robot for simple grasping sequences
'''
class UR10_robot_arm:

    '''
    TODO not done - is a simplified version that only saves position with a fixed preset orientation
    Reads the current robot TCP translation and rotation from the /tf topic that is published by the Universal Robots Driver
    Translation is x,y,z coordinates from the robots origin
    Rotation is the TCP orientation aka approach vector in quaternions
    Saves these internally in the class

    Parameters
        None
    Returns
        None
    Throws
        None
    '''
    def read_gripper_translation_rotation(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.frame_name, 'tool0_controller', rospy.Time())
            self.xTranslation = trans.transform.translation.x + self.gripper_offset
            self.yTranslation = trans.transform.translation.y + self.gripper_offset
            self.zTranslation = trans.transform.translation.z + self.gripper_offset


            a=tf_conversions.transformations.euler_from_quaternion((trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))

            #print('A',np.trunc(np.rad2deg(a)))
            #roll pitch yaw
            '''self.Rx,self.Ry, self.Rz = tf_conversions.transformations.euler_from_quaternion((trans.transform.rotation.x,
                                                                trans.transform.rotation.y,
                                                                trans.transform.rotation.z,
                                                                trans.transform.rotation.w))

            '''
            # TODO temp solution
            self.Rx = np.pi/2
            self.Ry = 0
            self.Rz = 0

            #print('qx:', trans.transform.rotation.x, 'qy:', trans.transform.rotation.y, 'qz:', trans.transform.rotation.z, 'qw:', trans.transform.rotation.w)

            v_yx = [np.cos(self.Rz), np.sin(self.Rz), 0]
            v_zx = [np.cos(self.Ry), 0,np.cos(self.Rz)]
            v_zy = [0, np.cos(self.Rx), np.sin(self.Rx)]

            v_r = np.add(v_yx, v_zx)
            v_r = np.add(v_r, v_zy)

            v_r_len = np.sqrt(v_r[0]**2 + v_r[1]**2 + v_r[2]**2)
            v_r = v_r / v_r_len

            self.direction_normal = v_r

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # print('not found trans yet')
            None

    # TODO proper comment, returns true when close enough to pos, pos in meter
    def is_at_position(self, x, y, z, Rx, Ry, Rz, tol_mm, tol_rad):
        self.read_gripper_translation_rotation()
        if math.isclose(self.xTranslation, x, abs_tol=tol_mm*0.001) and math.isclose(self.yTranslation, y, abs_tol=tol_mm*0.001) and math.isclose(self.zTranslation, z, abs_tol=tol_mm*0.001) and math.isclose(self.Rx, Rx, abs_tol=tol_rad) and math.isclose(self.Ry, Ry, abs_tol=tol_rad) and math.isclose(self.Rz, Rz, abs_tol=tol_rad):
            return True
        else:
            return False


    '''
    TODO - simplified version only moves in the right direction aka +X
    Moves the TCP endposition a certain distance to the right in relation the the gripper
    Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
    Parameters
        self
        distance_mm - distance to move ''right'' in relation to the gripper
    Returns
        None
    Throws
        None
    '''
    def move_gripper_R(self, distance_mm):
        # TODO to be able to calculate the moveL command we need to know orientation of gripper, put thought into
        # what joints are interesting to determine this. All probably? by calculating all joints we should be able to
        # know the direction the gripper is pointing, and then we read the hand position from the arm. Is there a
        # built in function for this?
        # By reading the orientation of the last joint we can turn that orthogonal plane into a line we can move along.
        # This way we would ensure that we always move in the direction we intend.
        # This would need to be divided into separate functions to make it tidy and functional.

        # TODO TEMP SOLUTION: ONLY MOVES RIGHT aka +x RELATIVE TO GLOABAL SYSTEM
        # points head forward
        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        # gripper orientation points it towards table
        newRx = np.pi / 2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation + distance_mm * 0.001  # converts to meters
        newY = self.yTranslation
        newZ = self.zTranslation

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    '''
    TODO - simplified version only moves in the LEFT direction aka -X
    Moves the TCP endposition a certain distance to the right in relation the the gripper
    Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
    Parameters
        self
        distance_mm - distance to move ''right'' in relation to the gripper
    Returns
        None
    Throws
        None
    '''
    def move_gripper_L(self, distance_mm):
        # same as above but in the other direction along calculated line.

        # TODO TEMP SOLUTION: ONLY MOVES LEFT aka -x RELATIVE TO GLOABAL SYSTEM
        # points head forward
        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()


        # gripper orientation points it towards table
        newRx = np.pi / 2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation - distance_mm * 0.001  # converts to meters
        newY = self.yTranslation
        newZ = self.zTranslation

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    '''
        TODO - simplified version only moves in the FORWARD direction aka -Y
        Moves the TCP endposition a certain distance to the right in relation the the gripper
        Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
        Parameters
            self
            distance_mm - distance to move ''right'' in relation to the gripper
        Returns
            None
        Throws
            None
        '''
    def move_gripper_forwards(self, distance_mm):
        # "only" need to move in the direction of the calculated orientation.
        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward

        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        # gripper orientation points it towards table
        newRx = np.pi / 2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation
        newY = self.yTranslation - distance_mm * 0.001  # converts to meters
        newZ = self.zTranslation

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    '''
        TODO - simplified version only moves in the BACKWARDS direction aka +Y
        Moves the TCP endposition a certain distance to the right in relation the the gripper
        Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
        Parameters
            self
            distance_mm - distance to move ''right'' in relation to the gripper
        Returns
            None
        Throws
            None
        '''
    def move_gripper_backwards(self, distance_mm):
        # "only" need to move against the direction of the calculated orientation.

        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward
        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        # gripper orientation points it towards table
        newRx = np.pi / 2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation
        newY = self.yTranslation + distance_mm * 0.001  # converts to meters
        newZ = self.zTranslation

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    '''
       TODO - simplified version only moves in the UP direction aka +Z
       Moves the TCP endposition a certain distance to the right in relation the the gripper
       Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
       Parameters
           self
           distance_mm - distance to move ''right'' in relation to the gripper
       Returns
           None
       Throws
           None
       '''

    def move_gripper_up(self, distance_mm):
        # move vertically, or move "up" from the grippers perspective? keep gripper at same direction?

        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward
        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

            # gripper orientation points it towards table
        newRx = np.pi / 2

        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation
        newY = self.yTranslation
        newZ = self.zTranslation + distance_mm * 0.001  # converts to meters

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)


    '''
    TODO - simplified version only moves in the DOWN direction aka -Z
    Moves the TCP endposition a certain distance to the right in relation the the gripper
    Does some calculations on where the new TCP position should be and then calls another method to execute the command to the robot
    Parameters
        self
        distance_mm - distance to move ''right'' in relation to the gripper
    Returns
        None
    Throws
        None
    '''


    def move_gripper_down(self, distance_mm):
        # same as up function
        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward

        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        # gripper orientation points it towards table
        newRx = np.pi / 2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation
        newY = self.yTranslation
        newZ = self.zTranslation - distance_mm * 0.001  # converts to meters

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    '''
    Moves the robot TCP to a wanted position with a wanted pose, moves linearly with a predefined low speed
    The position is in relation to the globabl coordinate system for the robot
    Parameters
        x  - position
        y - position
        z - position
        Rx - Rotation in  rad around X axix
        Ry - Rotation in rad around Y axis
        Rz - Rotation in rad around Z axis
    Returns

    Throws

    '''
    def move_position(self, x, y, z, Rx, Ry, Rz):
        self.execute_movel_cmd(x, y, z, Rx, Ry, Rz)

    '''
    Enabled the use of depth compensation when using the RG2-FT parallell jaw gripper, meaning it will move the TCP backwards when
    closing the gripper to keep the fingers locked in space.
    
    Does this by approximating the gripper jaw motion to a circle path and then calculating how much forwards/backwards it needs to move
    when opening/closing a certain distance
     
    Parameters
        current_width_mm - current jaw gripper width
        target_width_mm - target jaw gripper width
    Returns
        None
    Throws
        None
    '''
    def depth_compensation_gripper(self, current_width_mm, target_width_mm):
        #Moves the TCP forwards or backwards depending on how gripper is opening/closing
        #Krooks kod
        radious = 170
        current_depth_mm = radious - np.sqrt((radious**2 - current_width_mm**2))
        target_depth_mm = radious - np.sqrt((radious**2 - target_width_mm**2))

        self.execute_movel_cmd(self.xTranslation, self.yTranslation + current_depth_mm*0.001 - target_depth_mm*0.001, self.zTranslation, self.Rx, self.Ry, self.Rz)

        return True

    '''
    Executes a movel command to the robot by sending an movel command in urscript wich is then interpret by the robot
    moves linearly between where it currently is and where it should go
    WARNING: may generate paths that intersects the robot itself for longer distances
    
    Parameters
        x - pos
        y - pos
        z - pos
        Rx - rotation in rad around X axis
        Ry - rotation in rad around Y axis
        Rz - rotation in rad around Z axis
        a - acceleration in m/s*s
        v - velocity in m/s
        r - blend radius
    Returns
        None
    Throws
        None
    '''
        # Executes a command to the robot, prints in command prompt when sent
    def execute_movel_cmd(self,x, y, z, Rx, Ry, Rz, a=0.05, v=0.05,r=0):
        print('starting to execute command')
        movel_cmd = 'movel(p[' + str(x) + ',' + str(y) + ',' + str(z)+ ',' + str(Rx) + ',' + str(Ry) + ',' + str(Rz) + '],' + str(a) + ',' + str(v) + ',' + str(r) + ')'

        while not rospy.is_shutdown():
            connections = self.script_cmd_publisher.get_num_connections()
            if connections > 0:
                self.script_cmd_publisher.publish(movel_cmd)
                print('command has been executed : ', movel_cmd)
                break
            else:
                time.sleep(0.001)

    '''
    TODO - implement
    Moves the robot arm TCP to a predefined starting position with a wanted pose
    The starting position is choosen and hardcoded
    
    Parameters
        None
    Returns
        None
    Throws
        None

    '''
    # moves arm to a known starting position with a known pose
    def move_to_starting_pos(self):
        None

    '''
        Test sequence that moves the robot in a Line in the XY plane with length of 150 mm
        Used to verify functionality of methods created

        WARNING: using timers is not optimal, use the method is_at_position( ... ) to verify that it is in the right position instead

        Parameters
            None
        Returns
            None
        Throws
            None

        '''
    def move_forward_test_150_mm(self):
        while not rospy.is_shutdown():
            while self.xTranslation == 0:
                self.read_gripper_translation_rotation()

            print('xtrans:', self.xTranslation)

            self.read_gripper_translation_rotation()
            self.move_gripper_forwards(150)
            time.sleep(5)
            self.read_gripper_translation_rotation()
            self.move_gripper_backwards(150)
            time.sleep(5)


    '''
    Test sequence that moves the robot in a square in the XZ plane with sides of 200 mm
    Used to verify functionality of methods created
    
    WARNING: using timers is not optimal, use the method is_at_position( ... ) to verify that it is in the right position instead
    
    Parameters
        None
    Returns
        None
    Throws
        None

    '''
    def move_square_200_mm(self):
        while not rospy.is_shutdown():

            print('waiting for ok trans')

            while self.xTranslation == 0:
                self.read_gripper_translation_rotation()

            print('xtrans:', self.xTranslation)

            self.read_gripper_translation_rotation() #reads current robot poistion (not pose atm)
            self.move_gripper_up(200)
            time.sleep(10)
            self.read_gripper_translation_rotation()  # reads current robot poistion (not pose atm)
            self.move_gripper_R(200)
            time.sleep(10)
            self.read_gripper_translation_rotation()  # reads current robot poistion (not pose atm)
            self.move_gripper_down(200)
            time.sleep(10)
            self.read_gripper_translation_rotation()  # reads current robot poistion (not pose atm)
            self.move_gripper_L(200)
            time.sleep(10)

            print('successfuly moved in a square!!')


    '''
    Test method to check how well position can be read
    Paramters
        None
    Returns
        None
    Throws
        None
    '''
    def test(self):
        while not rospy.is_shutdown():

            print('waiting for ok trans')

            while self.xTranslation == 0:
                self.read_gripper_translation_rotation()

            print('xtrans:', self.xTranslation)



            #points head forward
            dir_x = np.pi / 2
            dir_y = 0
            dir_z = 0

            newX = self.xTranslation - 0.05
            newY = self.yTranslation
            newZ = self.zTranslation

            self.execute_movel_cmd(newX, newY, newZ, dir_x, dir_y, dir_z)

            #print('Albins x:',self.xTranslation, 'y:', self.yTranslation, 'z:', self.zTranslation, 'Rx:', np.rad2deg(self.Rx), 'Ry:', np.rad2deg(self.Ry), 'Rz:', np.rad2deg(self.Rz), ' OBS nu äre i grader')
            print('Data values from /tf: x', self.xTranslation, ' y', self.yTranslation, ' z', self.zTranslation)
            print('waiting 5 seconds... chill dude')

            self.read_gripper_translation_rotation() #reads new pos

            time.sleep(2)

    '''
    Constructor method that is called each time a new instance of this object is created
    Verifies that it has read a position before exiting init => all other methods can be used directly after creatign this object
    Parameters
        self
    Returns
        None
    Throws
        None

    '''

    def __init__(self):
       # rospy.init_node('armCtrl_node')
       # self.test_tf_grunkor()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frame_name = rospy.get_param('tool0_controller', 'base')
        self.script_cmd_publisher = rospy.Publisher('/ur_hardware_interface/script_command/', String, queue_size=1)

        self.xTranslation = 0
        self.yTranslation = 0
        self.zTranslation = 0
        self.Rx = 0
        self.Ry = 0
        self.Rz = 0
        self.direction_normal = [0, 0, 0]

        self.gripper_offset = 0 # vector length offset that tells how far from TCP the gripper TCP is at a given moment. Can change depending on how open/close the gripper is
        self.read_gripper_translation_rotation()

        self.script_cmd_publishers = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)

        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()


    '''
    Used for testing functionality, is never called upon and executed when creating instance of this object
    Parameters
        None
    Returns
        None
    Throws
        None

    '''
if __name__ == '__main__':
    rospy.init_node('test_depth_comp')
    arm1 = UR10_robot_arm()
    arm1.move_square_200_mm()

