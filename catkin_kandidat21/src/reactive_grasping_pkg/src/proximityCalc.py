#!/usr/bin/env python
# TODO make custom msgs to avoid string splcing !!!!

import rospy
from std_msgs.msg import String
import math

globa_msg = '[proximity_value_R=500,proximity_value_L=550]'
tolerance = 100


def prox_data_handler(msg):
    globa_msg = msg


# Get Proximity reading from left finger from gripperinterface publisher
# read string and splice out relevant data.
def get_prox_L():
    proxL = int(globa_msg.split('=', 3)[2].split(']')[0])

    #return mainController.validator_16(proxL)
    return proxL


# Get proximity reading from right finger
def get_prox_R():
    proxR = int(globa_msg.split('=',3)[1].split(',')[0])
    return proxR
  #  return mainController.validator_16(proxR)

# Check if Proximity readings match within tolerance, if not adjust position
# TODO This needs to access ROS node for controlling the arm to adjust position
def prox_check():
    # Run gripping command from main? or just return a "good" status, start closing grippers and then run proxCheck()
    # again once fingers are closer to object? interesting if we could with this grip a moving object?
    if math.isclose(get_prox_L(), get_prox_R(), abs_tol=tolerance):
        return print('Yay!') # ???

    elif (get_prox_L() - tolerance)> get_prox_R():
        # Move gripper right by getProxL() - getProxR() mm? SLOWLY! send this to armCalc
        None

    elif (get_prox_R() - tolerance) > get_prox_L():
        # Same as above but inverted send this to armCalc
        None


def main():
    rospy.init_node('topic_subscriber')
    proximity_sub = rospy.Subscriber('/gripper_interface/proximity_data/', String(), prox_data_handler) # listens to a topic and calls prox_data_handler
    rospy.spin()

if __name__ == '__main__':
    main()
    print('prox R ', get_prox_R())
    print('prox L ', get_prox_L())
    prox_check()