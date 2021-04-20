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

# only an example of easy implement for trial
def move_R(distance):
    # send moveJ base joint tiny tiny amount to right for starters?
    # do the same for move_L()
    None

#TODO descriptopm
#essentially this is the interface we can use to controll the arm easily by eg. armCalc.UR10_robot_arm.move_start_pos()
class UR10_robot_arm:

    def read_gripper_translation_rotation(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.frame_name, 'tool0_controller', rospy.Time())
            self.xTranslation = trans.transform.translation.x + self.gripper_offset
            self.yTranslation = trans.transform.translation.y + self.gripper_offset
            self.zTranslation = trans.transform.translation.z + self.gripper_offset


            a=tf_conversions.transformations.euler_from_quaternion((trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))

            print('A',np.trunc(np.rad2deg(a)))
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
            print('not found trans yet')



    # For now, move_gripper_R will move the arm in the direction of the finger marked R
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

        #its possible we want to
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


    def depth_compensation_gripper(self, current_width_mm, target_width_mm):
        #Moves the TCP forwards or backwards depending on how gripper is opening/closing
        #Krooks kod
        radious = 170
        current_depth_mm = radious - np.sqrt((radious**2 - current_width_mm**2))
        target_depth_mm = radious - np.sqrt((radious**2 - target_width_mm**2))

        self.execute_movel_cmd(self.xTranslation, self.yTranslation + abs(current_depth_mm - target_depth_mm)*0.001, self.zTranslation, self.Rx, self.Ry, self.Rz)

        None

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



    def move_gripper_up(self, distance_mm):
        # move vertically, or move "up" from the grippers perspective? keep gripper at same direction?

        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward
        self.read_gripper_translation_rotation()
        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        #gripper orientation points it towards table
        newRx = np.pi/2
        newRy = 0
        newRz = 0

        #gripper position
        newX = self.xTranslation
        newY = self.yTranslation
        newZ = self.zTranslation + distance_mm*0.001 #converts to meters

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    def move_gripper_down(self, distance_mm):
        # same as up function
        # TODO TEMP SOLUTION: ONLY MOVES Z UP RELATIVE TO GLOABAL SYSTEM
        # points head forward

        while self.xTranslation == 0 and self.yTranslation == 0 and self.zTranslation == 0:
            self.read_gripper_translation_rotation()

        # gripper orientation points it towards table
        newRx = np.pi/2
        newRy = 0
        newRz = 0

        # gripper position
        newX = self.xTranslation
        newY = self.yTranslation
        newZ = self.zTranslation - distance_mm * 0.001  # converts to meters

        self.execute_movel_cmd(newX, newY, newZ, newRx, newRy, newRz)

    # do we want functions that move the arm in a "absolute" direction? what is described above is all relative to the
    # gripper. These are needed to adjust position from the sensor feedback, but might be unnecessarily complicated
    # when moving larger distances or to rough positions for pickup/drop off.

    # moves arm to a known starting position with a known pose
    def move_to_starting_pos(self):
        None

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

    # init internal variables and start communication with the arm itself
    # some type of check to see if communication with arm is successfull
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

if __name__ == '__main__':
    rospy.init_node('test_depth_comp')
    arm1 = UR10_robot_arm()
    arm1.move_square_200_mm()

