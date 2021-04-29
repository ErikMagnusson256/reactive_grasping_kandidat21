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


'''
Class with helper functions to check slippage and then react to slippage by actuating the gripper
'''
class ForceCalcClass:
    '''
    Callback function that saves data that have been published on
    Parameters
        self
        msg - data that have been read from a topic
    Returns
        None
    Throws
        None
    '''
    def force_torque_data_handler(self, msg):

        self.current_msg = msg.data

    '''
    function that splices the Right force sensor data in X from a string
    
    Parameters
        self
    Returns
        force in right the right sensor in X dimension
    Throws
        None
    '''
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

# TODO debugg and understand why this happens

    def get_F_z_L(self):
        try:
            f_z_L = self.current_msg.split('=', )[6].split(']')[0]
            return int(f_z_L)
        except TypeError:
            print('f_z_L value is ', f_z_L, 'temporarily sends same value as f_z_R')

        finally:
            return self.get_F_z_R()

    '''
    helper function that can be called for to detect and react to slippage
    
    Parameters
        self
    Returns
        True - slip has been detected and reacted to
        False - no slip detected
    Throws
        None
    '''
    def slip_detect(self):
        #rospy.Subscriber('/gripper_interface/force_torque_data/', String, self.force_torque_data_handler)
        if (np.sqrt((self.get_F_x_L() ** 2) + (self.get_F_y_L() ** 2)) > abs((self.get_F_z_L() * 0.5))) or (
                np.sqrt((self.get_F_x_R() ** 2) + (self.get_F_y_R() ** 2)) > abs((self.get_F_z_R() * 0.5))):
            print('Caught me slipping')
            print(np.sqrt((self.get_F_x_L() ** 2) + (self.get_F_y_L() ** 2)), '>', (self.get_F_z_L() * 0.5))
            print(np.sqrt((self.get_F_x_R() ** 2) + (self.get_F_y_R() ** 2)), '>', (self.get_F_z_R() * 0.5))
            self.slip_react()
            time.sleep(0.1)
            return True
        else:
            return False, print('All good')

    '''
    TODO fix send single msg
    
    Function that reacts to slip by increasing gripping force and decreasing gripper width
    Createsa publisher to send cmd
    then sends cmd to actuate gripper
    
    Parameters
        self
    Returns
        None
    Throws
        None
    '''
    def slip_react(self):
        # gripperInterface1.operate_gripper_step_force(0.1)
        pub_cmd_forcecalc = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=1)
        # rospy.init_node('talkerdffdf', anonymous=True)

        while not rospy.is_shutdown():
            connections = pub_cmd_forcecalc.get_num_connections()
            print('watiing for connecting in forcecalc check slip react')
            if connections > 0:
                pub_cmd_forcecalc.publish('operate_gripper_step_force(5)')
                pub_cmd_forcecalc.publish('operate_gripper_step_width(1)')
                break

        time.sleep(0.05)

    '''
    Constructor, called each time new instance of object is created
    creates a subscriber to force data
    creates a variabe to store msgs that are being published on topic
    '''
    def __init__(self):
        self.current_msg = '[F_x_R=1,F_y_R=2,F_z_R=5,F_x_L=1,F_y_L=2,F_z_L=1]'
        rospy.Subscriber('/gripper_interface/force_torque_data/', String, self.force_torque_data_handler)

        #wait for real values

        while self.current_msg == '[F_x_R=1,F_y_R=2,F_z_R=5,F_x_L=1,F_y_L=2,F_z_L=1]':
            continue

        print('Found force values: FzL', self.get_F_z_L(), ' FzR', self.get_F_z_R())


def main():
    None


if __name__ == '__main__':
    main()
