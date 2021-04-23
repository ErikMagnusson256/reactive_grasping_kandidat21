#!/usr/bin/env python
# TODO comments and bös
import numpy as np

#from proximityCalc import ProximityCalcClass
import proximityCalc
import armCalc
from forceCalc import ForceCalcClass

import time

import rospy
from std_msgs.msg import String
from pymodbus.payload import BinaryPayloadDecoder

def stage_test():
    # add whatever you want to try, be smart about the order
    armm = armCalc.UR10_robot_arm()

    pub_cmd_maincontroller = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)


    #test to see hwo to send just one msg
    while not rospy.is_shutdown():
        connections = pub_cmd_maincontroller.get_num_connections()
        print('snurring')
        if connections > 0:
            pub_cmd_maincontroller.publish('operate_gripper_release')
            print('yay maincontroller sent a msg')
            break
        else:
            time.sleep(0.1)

    while not rospy.is_shutdown():
        forceLogic = ForceCalcClass()
        proximityLogic = proximityCalc.ProximityCalcClass()

        while not proximityLogic.prox_check():
            time.sleep(0.5)
            proximityLogic.prox_check()

        print('proximity done')

        armm.move_gripper_up(10)

        #if gripperInterface2.operate_gripper(gripperInterface1, 10, 1):
    # pub_cmd_maincontroller.publish('operate_gripper(40,10)')
        while forceLogic.slip_detect():
            time.sleep(0.5)
            forceLogic.slip_detect()
        time.sleep(5)



def grip_sequence(tolerance, solid):
    pub_cmd_maincontroller = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)

    # test to see hwo to send just one msg
    while not rospy.is_shutdown():
        connections = pub_cmd_maincontroller.get_num_connections()
        print('snurring')
        if connections > 0:
            pub_cmd_maincontroller.publish('operate_gripper_release')
            print('yay maincontroller sent a msg')
            break
        else:
            time.sleep(0.1)


    proximityLogic = proximityCalc.ProximityCalcClass()

    while not proximityLogic.prox_check(tolerance, solid):
        time.sleep(0.1)
        proximityLogic.prox_check(tolerance, solid)

    print('proximity done')
    time.sleep(0.5)

def slip_check():
    forceLogic = ForceCalcClass()
    while not forceLogic.slip_detect():
        time.sleep(0.01)
        forceLogic.slip_detect()


    return forceLogic.slip_detect()

def gripper_release():
    pub_cmd_maincontroller = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)

    # test to see hwo to send just one msg
    while not rospy.is_shutdown():
        connections = pub_cmd_maincontroller.get_num_connections()
        print('snurring')
        if connections > 0:
            pub_cmd_maincontroller.publish('operate_gripper_release')
            print('yay maincontroller sent a msg')
            break
        else:
            time.sleep(0.1)


def test_arm_pos():
    arm = armCalc.UR10_robot_arm()

    for i in range(10):
        arm.move_position(0.44, -0.7139, 0.0838, np.pi/2, 0, 0)
        while not arm.is_at_position(0.44, -0.7139, 0.0838, np.pi/2, 0, 0, 10, 0.1):
            continue
        print('at pos yo')
        arm.move_position(-0.6044, -0.8730, 0.0638, np.pi / 2, 0, 0)
        while not arm.is_at_position(-0.6044, -0.8730, 0.0638, np.pi/2, 0, 0, 10, 0.1):
            continue
        print('at pos yo')

def stage_0():
    #move to kopp be4 pcikupp
    #move forward to pickup
    # do grip sequence
    #done
    gripper_release()
    arm = armCalc.UR10_robot_arm()
    arm.move_position(0.0857, -0.7475, 0.0401, np.pi/2, 0, 0)
    while not arm.is_at_position(0.0857, -0.7475, 0.0331, np.pi/2, 0, 0, 10, 0.1):
        continue
    arm.move_gripper_forwards(55)
    while not arm.is_at_position(0.0857, -0.7475 - 0.055, 0.0331, np.pi / 2, 0, 0, 10, 0.1):
        continue
    grip_sequence(25, False)
    print('stage0 complete')
    None

def stage_1():
    # lift and check slip
    #move to dropoff and check slip onn the way
    arm = armCalc.UR10_robot_arm()

    arm.move_gripper_up(100)
    while not arm.is_at_position(0.0857, -0.7475 - 0.055, 0.0401 + 0.1, np.pi / 2, 0, 0, 30, 0.5):
        print('not at pos yo')
        slip_check()

    print('stage1 complete')
    None

def stage_2():
    #lower dto drop, do drop, happy. retract
    arm = armCalc.UR10_robot_arm()

    arm.move_position(-0.6044, -0.8730, 0.0401, np.pi/2, 0, 0)
    while not arm.is_at_position(-0.6044, -0.8730, 0.0401, np.pi / 2, 0, 0, 10, 0.5):
        slip_check()

    gripper_release()
    time.sleep(1)

    arm.move_gripper_backwards(100)
    while not arm.is_at_position(-0.6044, -0.8730 + 0.1, 0.0401, np.pi / 2, 0, 0, 50, 0.5):
        continue

    print('stage2 complete')
    None

def stage_3():
    # go to tyngd b4 pickup, move forward, grip sequence
    print('going to tygnd')
    gripper_release()
    arm = armCalc.UR10_robot_arm()

    arm.move_position(0.2724, -0.7581, 0.0120, np.pi/2, 0, 0)
    while not arm.is_at_position(0.2724, -0.7581, 0.0120, np.pi / 2, 0, 0, 10, 0.5):
        print('not at pos')
        continue
    print('at tynngd')
    arm.move_gripper_forwards(55)

    while not arm.is_at_position(0.2724, -0.7581 - 0.055, 0.0120, np.pi / 2, 0, 0, 10, 0.5):
        continue

    grip_sequence(5, True)
    print('stage3 complete')
    None

def stage_4():
    # lift and check slip, do what stage 1 does
    arm = armCalc.UR10_robot_arm()
    x = arm.xTranslation
    y = arm.yTranslation
    z = arm.zTranslation

    arm.move_gripper_up(10)
    while not arm.is_at_position(x, y, z + 0.01, np.pi / 2, 0, 0, 20, 0.5):
        slip_check()
    time.sleep(3)
    arm.move_gripper_up(100)
    while not arm.is_at_position(x, y, z + 0.1, np.pi / 2, 0, 0, 20, 0.5):
        slip_check()

    print('stage4 complete')
    None

def stage_5():
    #do stage 2
    arm = armCalc.UR10_robot_arm()
    arm.move_position(-0.6044, -0.8730, 0.0277, np.pi/2, 0, 0)


    while not arm.is_at_position(-0.6044, -0.8730, 0.0277, np.pi / 2, 0, 0, 10, 0.5):
        slip_check()

    gripper_release()
    time.sleep(1)
    arm.move_gripper_backwards(100)
    while not arm.is_at_position(-0.6044, -0.8730 + 0.1, 0.0277, np.pi / 2, 0, 0, 10, 0.5):
        continue

    print('stage5 complete')
    None

def stage_6():
    # go to trä, do stage 3
    arm = armCalc.UR10_robot_arm()
    arm.move_position(0.4472, -0.7139, 0.0638, np.pi / 2, 0, 0)
    while not arm.is_at_position(0.4472, -0.7139, 0.0638, np.pi / 2, 0, 0, 10, 0.5):
        continue


    arm.move_gripper_forwards(95)

    while not arm.is_at_position(0.4472, -0.7139 - 0.095, 0.0638, np.pi / 2, 0, 0, 10, 0.5):
        print('not in pos')
        continue

    grip_sequence(30, False)
    print('stage6 complete')
    None

def stage_7():
    # lift and check slip, do what stage 1 does
    arm = armCalc.UR10_robot_arm()
    arm.move_gripper_up(100)

    while not arm.is_at_position(0.4472, -0.7139 - 0.095, 0.0638 + 0.1, np.pi / 2, 0, 0, 10, 0.5):
        slip_check()

    print('stage7 complete')


def stage_8():
    # do stage 2. be done.
    arm = armCalc.UR10_robot_arm()
    arm.move_position(-0.6044, -0.8730, 0.0638, np.pi/2, 0, 0)

    while not arm.is_at_position(-0.6044, -0.8730, 0.0638, np.pi / 2, 0, 0, 10, 0.5):
        slip_check()

    gripper_release()
    time.sleep(1)

    arm.move_gripper_backwards(100)

    while not arm.is_at_position(-0.6044, -0.8730 + 0.1, 0.0638, np.pi / 2, 0, 0, 10, 0.5):
        continue

    print('stage8 complete')
    None

if __name__ == '__main__':

    rospy.init_node('talker_maincontroller', anonymous=True)
    stage_0()
    stage_1()
    stage_2()
    stage_3()
    stage_4()
    stage_5()
    stage_6()
    stage_7()
    stage_8()
    #test_arm_pos()

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

