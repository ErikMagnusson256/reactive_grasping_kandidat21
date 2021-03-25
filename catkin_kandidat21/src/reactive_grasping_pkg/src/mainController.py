#!/usr/bin/env python
# TODO comments and bös

import proximityCalc
import armCalc
import forceCalc
import gripperInterface

import rospy
from std_msgs.msg import String
from pymodbus.payload import BinaryPayloadDecoder

def stage_test():
    # add whatever you want to try, be smart about the order
    if proximityCalc.prox_chek():
        if gripperInterface.operate_gripper(10,0.5):
            while forceCalc.slip_detect():
                forceCalc.slip_detect()
            time.sleep(5)
            gripperInterface.operate_gripper_release()





def stage_0():
    # utilize something like armCalc.move_to_start()
    # make sure gripper is empty and open?

def stage_1():
    # move to rough object position, within reason? has to be able to pace object inside gripper
    # also orient gripper to desired attack angle? do b4 approaching object

def stage_2():
    # utilize proxCalc function to adjust into "perfect" position

def stage_3():
    # once proxCalc is happy go for the grip. Send grip command
    # check grip_detected register?
    # "pre flight checks" of force?

def stage_4():
    # liftoff, move arm and monitor forceCalc.slip_detect()
    # adjust force here or in forceCalc?

def stage_5():
    # move to desired drop off position, keep monitoring forceCalc.slip_detect()
    # do we want to have different "reactions" to slip here compared to slip at stage_4()?

def stage_6():
    # release object, disengage to "clear" the object and be ready to return to stage_0()

if __name__ == '__main__':
    None