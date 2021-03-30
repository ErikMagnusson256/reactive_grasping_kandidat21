# TODO description

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
import numpy as np
from gripperInterface import Rg2ftModbusROSInterface as gripperInterface1


#remove this later
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

class ForceCalcClass:

    def force_torque_data_handler(self, msg):
        self.current_msg = msg.data
        #print(self.current_msg)
        #print(self.get_F_z_L())

    def get_F_x_R(self):
        f_x_R = self.current_msg.split('=', 1)[1].split(',')[0]
        return int(f_x_R)

    def get_F_y_R(self):
        f_y_R = self.current_msg.split('=', )[2].split(',')[0]
        return int(f_y_R)

    def get_F_z_R(self):
        f_z_R = self.current_msg.split('=', )[3].split(',')[0]
        return int(f_z_R)

    def get_F_x_L(self):
        f_x_L = self.current_msg.split('=', )[4].split(',')[0]
        return int(f_x_L)

    def get_F_y_L(self):
        f_y_L = self.current_msg.split('=', )[5].split(',')[0]

        return int(f_y_L)

    def get_F_z_L(self):
        f_z_L = self.current_msg.split('=', )[6].split(']')[0]
        return int(f_z_L)

    def slip_detect(self):
        #rospy.Subscriber('/gripper_interface/force_torque_data/', String, self.force_torque_data_handler)

        if (np.sqrt((self.get_F_x_L() ** 2) + (self.get_F_y_L() ** 2)) > abs((self.get_F_z_L() * 0.5))) or (
                np.sqrt((self.get_F_x_R() ** 2) + (self.get_F_y_R() ** 2)) > abs((self.get_F_z_R() * 0.5))):
            print('Caught me slipping')
            print(np.sqrt((self.get_F_x_L() ** 2) + (self.get_F_y_L() ** 2)), '>', (self.get_F_z_L() * 0.5))
            print(np.sqrt((self.get_F_x_R() ** 2) + (self.get_F_y_R() ** 2)), '>', (self.get_F_z_R() * 0.5))
            self.slip_react()
            time.sleep(1)
            return True
        else:
            return False, print('All good')

    def slip_react(self):
        # gripperInterface1.operate_gripper_step_force(0.1)
        pub_cmd_forcecalc = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)
        # rospy.init_node('talkerdffdf', anonymous=True)
        # pub_cmd_forcecalc.publish('operate_gripper_step_force(1)')
        pub_cmd_forcecalc.publish('operate_gripper_step_width(5)')

    def __init__(self):
        self.current_msg = '[F_x_R=1,F_y_R=2,F_z_R=5,F_x_L=1,F_y_L=2,F_z_L=1]'
        rospy.Subscriber('/gripper_interface/force_torque_data/', String, self.force_torque_data_handler)



def main():
    #rospy.init_node('topic_subscriber')
  #  force_torque_sub = rospy.Subscriber('/gripper_interface/force_torque_data/', String, force_torque_data_handler)  # listens to a topic and calls prox_data_handler
    #rospy.spin()

    # while True:
    #     print("fxr:", get_F_x_R())
    #     print("fyr:", get_F_y_R())
    #     print("fzr:", get_F_z_R())
    #     print("fxl:", get_F_x_L())
    #     print("fyl:", get_F_y_L())
    #     print("fzl:", get_F_z_L())
    #     slip_detect()
    #     time.sleep(1)
    forceLogic = ForceCalcClass()
    forceLogic.get_F_z_L()


if __name__ == '__main__':
    main()
    # print("fxr:",get_F_x_R())
    # print("fyr:",get_F_y_R())
    # print("fzr:", get_F_z_R())
    # print("fxl:", get_F_x_L())
    # print("fyl:",get_F_y_L())
    # print("fzl:",get_F_z_L())
    # slip_detect()