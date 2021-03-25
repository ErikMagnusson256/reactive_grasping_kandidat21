#!/usr/bin/env python
#TODO add description of file

import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
import numpy as np

# only an example of easy implement for trial
def move_R(distance):
    # send moveJ base joint tiny tiny amount to right for starters?
    # do the same for move_L()
    None

#TODO descriptopm
#essentially this is the interface we can use to controll the arm easily by eg. armCalc.UR10_robot_arm.move_start_pos()
class UR10_robot_arm:

    #moves the tcp right according to markings on gripper
    def move_gripper_R(self, distance_mm):
        None

    def move_gripper_L(self, distance_mm):
        None

    def move_gripper_forwards(self, distance_mm):
        None

    def move_gripper_backwards(self, distance_mm):
        None

    def move_gripper_up(self, distance_mm):
        None

    def move_gripper_down(self, distance_mm):
        None

    # moves arm to a known starting position with a known pose
    def move_to_starting_pos(self):
        None

    #init internal variables and start communication with the arm itself
    # some type of check to see if communication with arm is successfull
    def __init__(self):
        None

if __name__ == '__main__':
    None