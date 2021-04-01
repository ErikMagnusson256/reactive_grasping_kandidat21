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

# only an example of easy implement for trial
def move_R(distance):
    # send moveJ base joint tiny tiny amount to right for starters?
    # do the same for move_L()
    None

#TODO descriptopm
#essentially this is the interface we can use to controll the arm easily by eg. armCalc.UR10_robot_arm.move_start_pos()
class UR10_robot_arm:

    def callback_tf_listener(self, msg):
        print('got a mail yo')
        if msg.data.child_frame_id == "tool0_controller":
            print('yay')

    def test_tf_grunkor(self):
        print('starting bös')
        sub_tf = rospy.Subscriber('listen_tf', tf2_msgs.msg.TFMessage, self.callback_tf_listener, queue_size=1 )
        while True:
            i = 1

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

    # init internal variables and start communication with the arm itself
    # some type of check to see if communication with arm is successfull
    def __init__(self):
        rospy.init_node('TEMP_armCtrl_tf_listener')
       # self.test_tf_grunkor()

if __name__ == '__main__':
    arm = UR10_robot_arm()
    arm.test_tf_grunkor()
