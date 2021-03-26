#!/usr/bin/env python
# TODO make custom msgs to avoid string splcing !!!!

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
import math
import gripperInterface

globa_msg = '[proximity_value_R=500,proximity_value_L=550]'
tolerance =45 #mm


# Decodes the data that has been retrived from registers
def validator_int16(instance):
    if not instance.isError():
        '''.isError() implemented in pymodbus 1.4.0 and above.'''
        decoder = BinaryPayloadDecoder.fromRegisters(
            instance.registers,
            byteorder=Endian.Big, wordorder=Endian.Little
        )
        return float('{0:.2f}'.format(decoder.decode_16bit_int()))

    else:
        # Error handling.
        print("There isn't the registers, Try again.")
        return None


def prox_data_handler(msg):
    global globa_msg
    globa_msg = msg.data



# Get Proximity reading from left finger from gripperinterface publisher
# read string and splice out relevant data.
def get_prox_L():
    proxL = globa_msg.split('=', )[2].split(']')[0]

    #return mainController.validator_16(proxL)
    return float(proxL)


# Get proximity reading from right finger
def get_prox_R():
    proxR = globa_msg.split('=', )[1].split(',')[0]
    return float(proxR) #in mm
  #  return mainController.validator_16(proxR)

# Check if Proximity readings match within tolerance, if not adjust position
# TODO This needs to access ROS node for controlling the arm to adjust position
def prox_check():
    #rospy.init_node('topic_subscriber_proxcalc')
    proximity_sub = rospy.Subscriber('/gripper_interface/proximity_data/', String, prox_data_handler)

    pub = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)
    #rospy.init_node('talkerdffdf', anonymous=True)
    # Run gripping command from main? or just return a "good" status, start closing grippers and then run proxCheck()
    # again once fingers are closer to object? interesting if we could with this grip a moving object?
    if math.isclose(get_prox_L(), get_prox_R(), abs_tol=tolerance):


        pub.publish('operate_gripper(42,1)')

        return True, print('Yay!', get_prox_L(), get_prox_R()) # ???

    # If distance from finger L + tolerance is larger than the distance from finger R, this mean we need to move
    # the arm in the direction of finger R. Standing on the "robots side", the finger marked as R is on the left,
    # sooo this means we need to decide from what perspective we are counting from.
    elif (get_prox_L() - tolerance)> get_prox_R():
        # Move gripper towards finger R by tiny amount depending on Hz? SLOWLY! send this to armCalc
        print('1')

    elif (get_prox_R() - tolerance) > get_prox_L():
        # Same as above but inverted send this to armCalc
        print('2')


def main():
    rospy.init_node('topic_subscriber')
    proximity_sub = rospy.Subscriber('/gripper_interface/proximity_data/', String, prox_data_handler) # listens to a topic and calls prox_data_handler
    #rospy.spin()

    while True:
        print('prox l:', get_prox_L(), ' prox r:', get_prox_R())

if __name__ == '__main__':
    main()
    print('prox R ', get_prox_R())
    print('prox L ', get_prox_L())
    prox_check()