#!/usr/bin/env python
# TODO comments and bös

import proximityCalc

import rospy
from std_msgs.msg import String
from pymodbus.payload import BinaryPayloadDecoder


# Decodes the data that has been retrived from registers
def validator_int16(instance):
    if not instance.isError():
        '''.isError() implemented in pymodbus 1.4.0 and above.'''
        decoder = BinaryPayloadDecoder.fromRegisters(instance.registers, byteorder=Endian.Big, wordorder=Endian.Little)
        return float('{0:.2f}'.format(decoder.decode_16bit_int()))

    else:
        # Error handling.
        print("There isn't the registers, Try again.")
        return None

def stage_X():
    proximityCalc.prox_check()

if __name__ == '__main__':
    None