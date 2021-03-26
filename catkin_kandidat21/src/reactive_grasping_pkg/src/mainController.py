#!/usr/bin/env python
# TODO comments and bös
import numpy

import proximityCalc
import armCalc
import forceCalc
from gripperInterface import Rg2ftModbusROSInterface as gripperInterface1
import time

import rospy
from std_msgs.msg import String
from pymodbus.payload import BinaryPayloadDecoder

def stage_test():
    # add whatever you want to try, be smart about the order
    rospy.init_node('talker_maincontroller', anonymous=True)
    pub = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)


    pub.publish('operate_gripper_release')

    while not proximityCalc.prox_check():
        proximityCalc.prox_check()



    #if gripperInterface2.operate_gripper(gripperInterface1, 10, 1):
    pub.publish('operate_gripper(40,10)')
    while forceCalc.slip_detect():
        forceCalc.slip_detect()
    time.sleep(5)







def stage_0():
    # utilize something like armCalc.move_to_start()
    # make sure gripper is empty and open?
    None

def stage_1():
    # move to rough object position, within reason? has to be able to pace object inside gripper
    # also orient gripper to desired attack angle? do b4 approaching object
    None

def stage_2():
    # utilize proxCalc function to adjust into "perfect" position
    None

def stage_3():
    # once proxCalc is happy go for the grip. Send grip command
    # check grip_detected register?
    # "pre flight checks" of force?
    None

def stage_4():
    # liftoff, move arm and monitor forceCalc.slip_detect()
    # adjust force here or in forceCalc?
    None

def stage_5():
    # move to desired drop off position, keep monitoring forceCalc.slip_detect()
    # do we want to have different "reactions" to slip here compared to slip at stage_4()?
    None

def stage_6():
    # release object, disengage to "clear" the object and be ready to return to stage_0()
    None

if __name__ == '__main__':
    stage_test()

class MainController:

    """constructor with printmessage for debugging"""
    def __init__(self):
        print('MainController constructor is called')

    """method converts degree to radian"""
    def degree_to_radian(self, degree):
        return numpy.deg2rad(degree)

    """method convert radian to degree"""
    def radian_to_degree(self, radian):
        return numpy.rad2deg(radian)

