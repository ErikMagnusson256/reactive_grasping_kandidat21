#!/usr/bin/env python

import numpy as np
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
# TODO solve comments and description

# @TODO Add proximity offsets when initing, double check that they have ben set

# testline
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


# Class to balblabllbalb
class Rg2ftModbusROSInterface:

    # Unit addresses
    rg2ft_device_addr = 65
    comp_box_device_addr = 63

    # Internal register addresses on which is stored and can be read to and/or written to
    zero_values_addr = 0  # Zero, Read+Write
    target_force_addr = 2  # Target force, Write
    target_width_addr = 3  # Target width, Write
    control_addr = 4  # Control (execute movement if 1, else 0), Write
    proximity_offset_L_addr = 5  # Read+Write
    proximity_offset_R_addr = 6  # Read+Write
    status_L_addr = 257  # Read
    F_x_L_addr = 259  # Read
    F_y_L_addr = 260
    F_z_L_addr = 261
    T_x_L_addr = 262
    T_y_L_addr = 263
    T_z_L_addr = 264
    status_R_addr = 266
    F_x_R_addr = 268
    F_y_R_addr = 269
    F_z_R_addr = 270
    T_x_R_addr = 271
    T_y_R_addr = 272
    T_z_R_addr = 273
    proximity_sensor_status_L_addr = 274
    proximity_value_L_addr = 275
    proximity_sensor_status_R_addr = 277
    proximity_value_R_addr = 278
    gripper_width_addr = 280
    gripper_busy_addr = 281
    grip_detected_addr = 282

    # Get data from registers regarding proximity and publishes on topic as a string
    def get_proximity_registers(self):
        prox_val_L = validator_int16(self.client.read_holding_registers(self.proximity_value_L_addr, 1, unit=self.rg2ft_device_addr))
        prox_val_R = validator_int16(self.client.read_holding_registers(self.proximity_value_R_addr, 1, unit=self.rg2ft_device_addr))

        return '[proximity_value_R='+str(prox_val_R)+',proximity_value_L='+str(prox_val_L)+']'

    # Get data from registes regarding force and torque and publishes on topic as string
    def get_force_registers(self):
        # TODO add torque  readings later on

        F_x_R = validator_int16(self.client.read_holding_registers(self.F_x_R_addr, 1, unit=self.rg2ft_device_addr))
        F_y_R = validator_int16(self.client.read_holding_registers(self.F_y_R_addr, 1, unit=self.rg2ft_device_addr))
        F_z_R = validator_int16(self.client.read_holding_registers(self.F_z_R_addr, 1, unit=self.rg2ft_device_addr))
        F_x_L = validator_int16(self.client.read_holding_registers(self.F_x_L_addr, 1, unit=self.rg2ft_device_addr))
        F_y_L = validator_int16(self.client.read_holding_registers(self.F_y_L_addr, 1, unit=self.rg2ft_device_addr))
        F_z_L = validator_int16(self.client.read_holding_registers(self.F_z_L_addr, 1, unit=self.rg2ft_device_addr))
        return '[F_x_R=' + str(F_x_R) + ',F_y_R=' + str(F_y_R) + ',F_z_R=' + str(F_z_R) + ',F_x_L=' + str(F_x_L) + ',F_y_L=' + str(F_y_L) + ',F_z_L' + str(F_z_L)+']'

    # Get data from registers regarding misc information, eg: gripper status
    def get_miscellaneous_registers(self):
        None


    def generate_graphs(self, mode, n_datapoints, delta_t):

        if mode == 1:
            #prox grapho
            prox_data_R = []
            prox_data_L = []
            gripper_width_data = []

            #TODO add offset globaly in gripperitnerface
            offset_right = -(27)
            offset_left = -(17)

            self.client.write_register(self.target_width_addr, 400, unit=self.rg2ft_device_addr)
            self.client.write_register(self.target_force_addr, 100, unit=self.rg2ft_device_addr)
            self.client.write_register(self.control_addr, 1, unit=self.rg2ft_device_addr)

            for i in range(n_datapoints):
                prox_data_L.append(validator_int16(self.client.read_holding_registers(self.proximity_value_L_addr, 1, unit=self.rg2ft_device_addr))/10 + offset_left)
                prox_data_R.append(validator_int16(self.client.read_holding_registers(self.proximity_value_R_addr, 1, unit=self.rg2ft_device_addr))/10 + offset_right)
                gripper_width_data.append(validator_int16(self.client.read_holding_registers(self.gripper_width_addr, 1, unit=self.rg2ft_device_addr))/10)


                time.sleep(delta_t) #10ms

            x = [a for a in range(n_datapoints)]
            plt.plot(x, prox_data_L)
            plt.plot(x, prox_data_R)
            plt.plot(x, gripper_width_data)
            plt.legend(['prox_L','prox_R','gripper_width'])
            plt.yticks(np.arange(0, 100, 2), fontsize=8)
            x_lab = 'x - delta time between datapoints: '+str(delta_t) + ' s'
            plt.xlabel(x_lab)
            plt.ylabel('[mm]')
            #plt.xticks(np.arrange(0,n_datapoints, 50))

            print('pringitng average prox l:', np.average(prox_data_L))
            print('pringitng average prox R:', np.average(prox_data_R))
            plt.show()

    def generate_graphs_force(self, mode, n_datapoints, delta_t):

            if mode == 1:
                # prox grapho
                force_data_R = []
                force_data_L = []
                #gripper_width_data = []



                for i in range(n_datapoints):
                    force_data_L.append(validator_int16(
                        self.client.read_holding_registers(self.F_z_L_addr, 1,
                                                           unit=self.rg2ft_device_addr)) / 10 )
                    force_data_R.append(validator_int16(
                        self.client.read_holding_registers(self.F_z_R_addr, 1,
                                                           unit=self.rg2ft_device_addr)) / 10 )
                   # gripper_width_data.append(validator_int16(
                  #      self.client.read_holding_registers(self.gripper_width_addr, 1,
                    #                                       unit=self.rg2ft_device_addr)) / 10)

                    time.sleep(delta_t)  # 10ms

                x = [a for a in range(n_datapoints)]
                plt.plot(x, force_data_L)
                plt.plot(x, force_data_R)

                plt.legend(['Force_L', 'Force_R'])
                plt.yticks(np.arange(0, 100, 2), fontsize=8)
                x_lab = 'x - delta time between datapoints: ' + str(delta_t) + ' s'
                plt.xlabel(x_lab)
                plt.ylabel('[mm]')
                # plt.xticks(np.arrange(0,n_datapoints, 50))

                print('pringitng average prox l:', np.average(force_data_L))
                print('pringitng average prox R:', np.average(force_data_R))
                plt.show()


            elif mode == 2:
            #force/torque graph
                None
            else:
                print('no mode selected')



    # while loop, continously reading data and publishes those on certain topics
    def run(self):

        #OBS REMOVE
        self.operate_gripper(60, 10)

        while not rospy.is_shutdown():
            # Reads from registers and turns them into strings
            force_data = self.get_force_registers()
            prox_data = self.get_proximity_registers()
            misc_data = self.get_miscellaneous_registers()

            #publishes strings on topics
            self.pub_force.publish(force_data)
            self.pub_proximity.publish(prox_data)
          #  self.pub_misc(misc_data)


            #TODO fix
            time.sleep(1)

        self.client.close()

    #TODO gör detta till en service, tar nu endast emot kommandon på ett topic liksom
    # eg cmd 'operate_gripper(50, 60)'
    #        'operate_gripper_step_width(10)'
    #        'operate_gripper_step_force(1)'
    #        'operate_gripper_release'
    def gripper_cmd_handler(self, msg):
        cmd_string = msg.data

        # check what type of cmd that has been sent, see function method to see examples of allowable

        if cmd_string.contains('operate_gripper('):
            print('Command recived from topic /gripper_interface/gripper_cmd:', cmd_string,': operating gripper executing')
            width = int(cmd_string.split('(')[1].split(',')[0])
            force = int(cmd_string.split(',')[1].split(')')[0])
            self.operate_gripper(width,force)


        elif cmd_string.contains('operate_gripper_step_width('):
            print('Command recived from topic /gripper_interface/gripper_cmd:', cmd_string, ': operating_gripper_step_width executing')
            step_width = cmd_string.split('(')[1].split(')')[0]
            self.operate_gripper_step_width(step_width)

        elif cmd_string.contains('operate_gripper_step_force('):
            print('Command recived from topic /gripper_interface/gripper_cmd:', cmd_string, ': operating_gripper_step_force executing')
            step_force = cmd_string.split('(')[1].split(')')[0]
            self.operate_gripper_step_force(step_force)

        elif cmd_string.contains('operate_gripper_release'):
            print('Command recived from topic /gripper_interface/gripper_cmd:', cmd_string, ': operate_gripper_release executing')
            self.opeate_grippper_release()

        else:
            print("Unknown command recived on /gripper_interface/gripper_cmd/ :", cmd_string)


    def operate_gripper_step_width(self, increment_width_mm):
       None

    def operate_gripper_step_force(self, increment_force_Nm):
        None

    def opeate_grippper_release(self):
        None

    #open/closes gripper to a certain width with a set force
    def operate_gripper(self,grip_width_mm, grip_force_Nm):
        # 1 wait for gripper to not be busy
        while validator_int16(read_holding_registers(self.gripper_busy_addr, 1, unit=self.rg2ft_device_addr)) == 1:
            continue

        # 2 set target width and force in registers, multiply with ten to convert it to data that the gripper understands
        self.client.write_register(self.target_width_addr, grip_width_mm*10, unit=self.rg2ft_device_addr)
        self.client.write_register(self.target_force_addr, grip_force_Nm*10, unit=self.rg2ft_device_addr)

        # 3 execute gripper commandprint('Command recived from topic /gripper_interface/gripper_cmd, executing operate_gripper_step_(...)')
        self.client.write_register(self.control_addr, 1, unit=self.rg2ft_device_addr)

        # 4 wait for gripper to not be busy
        while validator_int16(read_holding_registers(self.gripper_busy_addr, 1, unit=self.rg2ft_device_addr)) == 1:
            continue

        #grip executed successfully
        return True

    def __init__(self):
        box_ip = "192.168.1.1"  # OnRobot computebox IP address
        self.client = ModbusClient(box_ip, port=502)  # Creates a client in this program that uses the box ip and port 502 (as per standard for modbus)
        return_val = self.client.connect()  # Tries to connect to the computebox
        print("Established connection to compute box?: ", return_val)  # prints in console if a connection is established
        if return_val:
            # init publishers
            rospy.init_node('topic_publisher')
            self.pub_proximity = rospy.Publisher('/gripper_interface/proximity_data/', String, queue_size=1)
            self.pub_force = rospy.Publisher('/gripper_interface/force_torque_data/', String, queue_size=1)
            self.pub_misc = rospy.Publisher('/gripper_interface/misc_data', String, queue_size=1)
          #  rospy.init_node('topic_subscriber')
           # self.sub_gripper_cmd('gripper_interface/gripper_cmd/', self.gripper_cmd_handler)
            publishing_rate_Hz = rospy.Rate(1)

            # init service???
        else:
            print("Catastrophic error please check yo self!!")

if __name__ == '__main__':

    C = Rg2ftModbusROSInterface()
    #C.generate_graphs(1, 200, 0.005)
    C.generate_graphs_force(1, 100, 0.1)
    #C.run()


