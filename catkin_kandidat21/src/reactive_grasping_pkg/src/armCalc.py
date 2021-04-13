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
            self.Rx,self.Ry, self.Rz = tf_conversions.transformations.euler_from_quaternion((trans.transform.rotation.x,
                                                                trans.transform.rotation.y,
                                                                trans.transform.rotation.z,
                                                                trans.transform.rotation.w))
            print('qx:', trans.transform.rotation.x, 'qy:', trans.transform.rotation.y, 'qz:', trans.transform.rotation.z, 'qw:', trans.transform.rotation.w)

            v_yx = [np.cos(self.Rz), np.sin(self.Rz), 0]
            v_zx = [np.cos(self.Ry), 0,np.cos(self.Rz)]
            v_zy = [0, np.cos(self.Rx), np.sin(self.Rx)]

            v_r = np.add(v_yx, v_zx)
            v_r = np.add(v_r, v_zy)

            v_r_len = np.sqrt(v_r[0]**2 + v_r[1]**2 + v_r[2]**2)
            v_r = v_r / v_r_len

            self.direction_normal = v_r

            #direction vector that the gripper is pointing in, normalised, length = 1
            #self.direction_normal = vectors.Vector(np.cos(self.Rz), np.cos(self.Rx), np.cos(self.Ry))
            #self.direction_normal = vectors.Vector(np.cos(self.Rz)*np.cos(self.Ry), np.sin(self.Rz)*np.cos(self.Ry), np.sin(self.Ry))
            #self.direction_normal = self.direction_normal.multiply(1/self.direction_normal.magnitude())
            print('v1:', v_yx, 'v2:', v_zx, 'v3:', v_zy)
            print('direction normal:', (self.direction_normal), 'len', np.sqrt(v_r[0]**2 + v_r[1]**2 + v_r[2]**2))

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
        None

    def move_gripper_L(self, distance_mm):
        # same as above but in the other direction along calculated line.
        None

        #its possible we want to
    def move_gripper_forwards(self, distance_mm):
        # "only" need to move in the direction of the calculated orientation.
        None

    def move_gripper_backwards(self, distance_mm):
        # "only" need to move against the direction of the calculated orientation.
        dir_vec = [-1*self.Rx, -1*self.Ry, -1*self.Rz]
        len_dir_vec = math.sqrt(self.Rx**2 + self.Ry**2, self.Rz**2)
        normalised_dir_vec = dir_vec / len_dir_vec
        move_distance = 0.1 #10 cm

        newX = self.xTranslation + normalised_dir_vec[0]*move_distance
        newY = self.yTranslation + normalised_dir_vec[1]*move_distance
        newZ = self.zTranslation + normalised_dir_vec[2]*move_distance

    def execute_movel_cmd(self,x, y, z, Rx, Ry, Rz, a=0.05, v=0.05,r=0,t=0):
        None

    def move_gripper_up(self, distance_mm):
        # move vertically, or move "up" from the grippers perspective? keep gripper at same direction?
        None

    def move_gripper_down(self, distance_mm):
        # same as up function
        None

    # do we want functions that move the arm in a "absolute" direction? what is described above is all relative to the
    # gripper. These are needed to adjust position from the sensor feedback, but might be unnecessarily complicated
    # when moving larger distances or to rough positions for pickup/drop off.

    # moves arm to a known starting position with a known pose
    def move_to_starting_pos(self):
        None

    def test(self):
        while not rospy.is_shutdown():
            self.read_gripper_translation_rotation()

            #print('Albins x:',self.xTranslation, 'y:', self.yTranslation, 'z:', self.zTranslation, 'Rx:', np.rad2deg(self.Rx), 'Ry:', np.rad2deg(self.Ry), 'Rz:', np.rad2deg(self.Rz), ' OBS nu äre i grader')
            #print('waiting 1 seconds... chill dude')
            time.sleep(1)

    # init internal variables and start communication with the arm itself
    # some type of check to see if communication with arm is successfull
    def __init__(self):
        rospy.init_node('armCtrl_node')
       # self.test_tf_grunkor()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frame_name = rospy.get_param('tool0_controller', 'base')

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

if __name__ == '__main__':
    arm = UR10_robot_arm()
    arm.test()
