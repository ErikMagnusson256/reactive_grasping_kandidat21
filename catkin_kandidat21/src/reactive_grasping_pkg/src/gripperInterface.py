#!/usr/bin/env python

import numpy as np
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import rospy
import std_msgs.msg
from std_msgs.msg import String
import time

'''
Method to translate hex values that have been read from modbus registers 
into decimal  numbers. Does check that instance is valid. Prints error 
message if it is not

Parameters
    instance - data that have been read from modbus registers
    
Returns
    Data from from registers as integer

Throws 
    None
'''
def validator_int16(instance):

    if not instance.isError():
        '''.isError() implemented in pymodbus 1.4.0 and above.'''
        decoder = BinaryPayloadDecoder.fromRegisters(
            instance.registers,
            byteorder=Endian.Big, wordorder=Endian.Little
        )
        return int(float('{0:.2f}'.format(decoder.decode_16bit_int())))

    else:
        # Error handling.
        print("There isn't the registers, Try again.")
        return None


'''
Class to represent the interface between Robot Operating System and the 
modbus interface. Is a middleman between ROS and the Computebox, which 
in turn controls the RG2-FT gripper.
Works as a server, continously sends and read data
'''
class Rg2ftModbusROSInterface:

    '''
    Unit addresses that are avalible inside the computebox, specifies
    which hardware to contact and read registers from
    '''
    rg2ft_device_addr = 65
    comp_box_device_addr = 63


    '''
    Internal register addresses on which is stored and can be read to 
    and/or written to inside the RG2-FT gripper
    '''
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


    '''
    Reads proximity data from RG2-FT gripper through modbus and then saves 
    the data in a string
    Reads left and right gripper proximity data
    Parameters
        self
    Returns
        Returns a string with data in a set order
    Throws
        None
    '''
    def get_proximity_registers(self):
        try:
            prox_val_L = validator_int16(
                self.client.read_holding_registers(self.proximity_value_L_addr,
                                                   1, unit=self.rg2ft_device_addr))
            prox_val_R = validator_int16(
                self.client.read_holding_registers(self.proximity_value_R_addr,
                                                   1, unit=self.rg2ft_device_addr))

            if prox_val_R == None or prox_val_L == None:
                return '[proximity_value_R=' + str(self.last_prox_R) + \
                       ',proximity_value_L=' + str(
                    self.last_prox_L) + ']'

            self.last_prox_L = prox_val_L
            self.last_prox_R = prox_val_R
            return '[proximity_value_R='+str(prox_val_R)+',proximity_value_L=' + \
                   str(prox_val_L)+']'
        except:
            print('Could not read proximity registers, using last known values R L:',
                  self.last_prox_R, self.last_prox_L)
            return '[proximity_value_R='+str(self.last_prox_R)+',proximity_value_L='+\
                   str(self.last_prox_L)+']'

    '''
       Reads current gripper width data from RG2-FT gripper through modbus and then 
       saves the data in a string
       Reads left and right gripper proximity data
       Parameters
           self
       Returns
           Returns a string with data in a set order
    Throws
            None
    '''
    def get_current_width_registers(self):
        try:
            current_width = validator_int16(
                self.client.read_holding_registers(self.gripper_width_addr,
                                                   1, unit=self.rg2ft_device_addr))

            if  current_width == None:
                return self.last_current_width

            self.last_current_width = current_width

            return str(current_width)
        except:
            print('Could not retrive current width, using last known current width:',
                  self.last_current_width)
            return self.last_current_width

    '''
       Reads force data from RG2-FT gripper through modbus and then saves the 
       data in a string
       Reads left and right gripper proximity data
       Parameters
           self
       Returns
           Returns a string with data in a prefix
        Throws
            None
        '''
    def get_force_registers(self):
        try:
            F_x_R = validator_int16(self.client.read_holding_registers(
                self.F_x_R_addr, 1, unit=self.rg2ft_device_addr))
            F_y_R = validator_int16(self.client.read_holding_registers(
                self.F_y_R_addr, 1, unit=self.rg2ft_device_addr))
            F_z_R = validator_int16(self.client.read_holding_registers(
                self.F_z_R_addr, 1, unit=self.rg2ft_device_addr))
            F_x_L = validator_int16(self.client.read_holding_registers(
                self.F_x_L_addr, 1, unit=self.rg2ft_device_addr))
            F_y_L = validator_int16(self.client.read_holding_registers(
                self.F_y_L_addr, 1, unit=self.rg2ft_device_addr))
            F_z_L = validator_int16(self.client.read_holding_registers(
                self.F_z_L_addr, 1, unit=self.rg2ft_device_addr))

            if F_x_L == None or F_x_R == None or F_y_L == None or F_y_R == None \
                    or F_z_L == None or F_z_R == None:
                return '[F_x_R=' + str(self.last_F_x_R) + ',F_y_R=' \
                       + str(self.last_F_y_R) + ',F_z_R=' + \
                       str(self.last_F_z_R) + ',F_x_L=' + str(self.last_F_x_L) + \
                       ',F_y_L=' + str(self.last_F_y_L) + \
                       ',F_z_L=' + str(self.last_F_z_L) + ']'

            self.last_F_x_R = F_x_R
            self.last_F_y_R = F_y_R
            self.last_F_z_R = F_z_R
            self.last_F_x_L = F_x_L
            self.last_F_y_L = F_y_L
            self.last_F_z_L = F_z_L

            return '[F_x_R=' + str(F_x_R) + ',F_y_R=' + str(F_y_R) + ',F_z_R=' + \
                   str(F_z_R) + ',F_x_L=' + str(
                F_x_L) + ',F_y_L=' + str(F_y_L) + ',F_z_L=' + str(F_z_L) + ']'
        except:
            print('Could not read force registers, using last known variables '
                  'F_x_R F_y_R F_z_R F_x_L F_y_L F_z_L:',
                  self.last_F_x_R, self.last_F_y_R, self.last_F_z_R, self.last_F_x_L,
                  self.last_F_y_L, self.last_F_z_L)
            return '[F_x_R=' + str(self.last_F_x_R) + ',F_y_R=' + str(self.last_F_y_R) +\
                   ',F_z_R=' +\
                   str(self.last_F_z_R) + ',F_x_L=' + str(self.last_F_x_L) + ',F_y_L=' +\
                   str(self.last_F_y_L) +\
                   ',F_z_L=' + str(self.last_F_z_L) + ']'

    '''
    @TODO 
    
    Parameters
    
    Returns
    
    Throws
    '''
    # Get data from registers regarding misc information, eg: gripper status
    def get_miscellaneous_registers(self):
        None


    '''
    A continous loop that reads data from registers and publishes data on set topics
    Starts when creating class instance through init method
    Publishing rate 1 Hz - TODO FIX
    
    Parameters
        self
    Returns
        None
    Throws
        None
    '''
    def run(self):

        while not rospy.is_shutdown():
            time1 = time.time()

            #check if there has been a new cmd to gripper
            if self.has_stored_cmd and self.cmd_string != None:
                self.gripper_cmd_execution()
                self.has_stored_cmd = False

            #waits for the gripper to execute commands before starting to read data
            while self.is_writing_registers and not self.is_reading_registers:
                continue

            # icurrently reading data from registers, don't want to accidentaly write
            # to them at the same time
            self.is_reading_registers = True

            # Reads from registers and turns them into strings
            force_data = self.get_force_registers()
            prox_data = self.get_proximity_registers()
            misc_data = self.get_miscellaneous_registers()
            c_w = self.get_current_width_registers()

            #publishes strings on topics
            self.pub_force.publish(force_data)
            self.pub_proximity.publish(prox_data)
            self.pub_width.publish(c_w)
            #  self.pub_misc(misc_data)

            self.is_reading_registers = False
            #TODO fix
            time2 = time.time()

            dt = 1/self.wanted_hz
            delta_dt = dt - (time2 - time1)
            if delta_dt < 0:
                delta_dt = 0
            time.sleep(delta_dt)





            #print('delta t:', time2 - time1, 'now sleeping:', delta_dt, 'seconds')

            if time2 - time1 > 1:
                print('ERROR WATING MOER THAN 1s', time2 - time1)

        self.client.close()


    '''
    Callback function that is executed when a new message is published on 
    /gripper_interface/gripper_cmd/ topic
    Data from this topic can contain commands that this class can execute to 
    actuate the gripper
    eg cmd 'operate_gripper(50, 60)'
           'operate_gripper_step_width(10)'
            'operate_gripper_step_force(1)'
            'operate_gripper_release'
    TODO - make this to a service
    Parameters
        self
        msg - data that have been published on the gripper_interface/gripper_cmd/ topic
    Returns
        None
    Throws 
        None
    '''
    def gripper_cmd_handler(self, msg):
        self.cmd_string = msg.data
        self.has_stored_cmd = True

    def gripper_cmd_execution(self):


        # check what type of cmd that has been sent, see function method to see
        # examples of allowable
        self.write_cmd_prio = True
        if 'operate_gripper(' in self.cmd_string :
            print('Command recived from topic /gripper_interface/gripper_cmd:',
                  self.cmd_string ,': operating gripper executing')
            width = int(self.cmd_string .split('(')[1].split(',')[0])
            force = int(self.cmd_string .split(',')[1].split(')')[0])

            print('width', width, ' force', force)
            self.operate_gripper(width,force)
            print('done operate gripper')

        elif 'operate_gripper_step_width(' in self.cmd_string :
            print('Command recived from topic /gripper_interface/gripper_cmd:',
                  self.cmd_string , ': operating_gripper_step_width executing')
            step_width = int(self.cmd_string .split('(')[1].split(')')[0])
            print('step_width:',step_width)
            self.operate_gripper_step_width(step_width)
            print('done step width')

        elif 'operate_gripper_step_force(' in self.cmd_string :
            print('Command recived from topic /gripper_interface/gripper_cmd:',
                  self.cmd_string , ': operating_gripper_step_force executing')
            step_force = int(self.cmd_string .split('(')[1].split(')')[0])
            print('step_force:', step_force)
            self.operate_gripper_step_force(step_force)
            print('done step force')

        elif 'operate_gripper_release' in self.cmd_string :
            print('Command recived from topic /gripper_interface/gripper_cmd:',
                  self.cmd_string , ': operate_gripper_release executing')
            self.opeate_grippper_release()
            print('done release')

        else:
            print("Unknown command recived on /gripper_interface/gripper_cmd/ :",
                  self.cmd_string )

        self.write_cmd_prio = False



    '''
    Reads the current width and calculates new width for gripper to close it by a set 
    step amount Calls operate_gripper( , ) to actuate the gripper with calculated width 
    and with same force as last time it gripped
    Parameters
        increment_width_mm - step width to close gripper with [mm]
    Returns 
        None
    Throws
        None
    '''
    def operate_gripper_step_width(self, increment_width_mm):
        current_width = self.get_current_width()/10
        print('operate gripper step width => curent width:', current_width,
              ' inc width:', increment_width_mm)

        self.operate_gripper((current_width - increment_width_mm), self.current_force)

    '''
       Reads the last used force and calculates new force for gripper to increase grip 
       force it by a set step amount Calls operate_gripper( , ) to actuate the gripper with 
       calculated new force and with same width as last time it gripped
       Parameters
           increment_force_N - step force to close gripper with [N]
       Returns 
           None
       Throws
           None
       '''
    def operate_gripper_step_force(self, increment_force_N):
        current_width = self.get_current_width()/10
        self.current_force = self.current_force + increment_force_N
        if self.current_force > 40:
            self.current_force = 40

        self.operate_gripper(current_width, (self.current_force))

    '''
       Opens the gripper fully with a set force of 10 N 
       saves the new force used
       Calls operate_gripper( , ) to actuate the gripper
       Parameters
           self
       Returns 
           None
       Throws
           None
       '''
    def opeate_grippper_release(self):
        self.operate_gripper(100, 10)
        self.current_force = 10
        return True

    '''
    Reads the current width from the registers
    
    Parameters
        self
    returns
        current_width - 
    Throws
        None
    '''
    def get_current_width(self):
        try:
            current_width = int(validator_int16(
                self.client.read_holding_registers(self.gripper_width_addr,
                                                   1, unit=self.rg2ft_device_addr)))
            self.last_current_width = current_width
            return int(current_width)
        except:
            print('Could not retrive current width, using last known current width:',
                  self.last_current_width)
            return self.last_current_width

    '''
    Actuates the gripper by writing data to certain registers in the gripper, 
    is done through modbus
    
    Parameters
        grip_width_mm - width in mm
        grip_force_N - force in N
    returns
        True - when finnished executing
    Throws
        None
    '''
    def operate_gripper(self,grip_width_mm, grip_force_N):

        #waits for reading registers to be done before writing things
        while self.is_reading_registers:
            continue

        self.is_writing_registers = True

        # 1 wait for gripper to not be busy
        while (self.client.read_holding_registers(
                self.gripper_busy_addr, 1, unit=self.rg2ft_device_addr)) == 1:
            continue

        # 2 set target width and force in registers,
        # multiply with ten to convert it to
        # data that the gripper understands
        self.client.write_register(self.target_width_addr, int(grip_width_mm*10),
                                   unit=self.rg2ft_device_addr)
        self.client.write_register(self.target_force_addr, int(grip_force_N*10),
                                   unit=self.rg2ft_device_addr)

        print('wrote:',int(grip_width_mm*10),' to', self.target_width_addr)
        print('wrote:', int(grip_force_N*10),' to', self.target_force_addr)

        # 3 execute gripper command
        self.client.write_register(self.control_addr, 1, unit=self.rg2ft_device_addr)

        # Save the force passed
        self.current_force = grip_force_N

        # 4 wait for gripper to not be busy
        while (self.client.read_hexecuteolding_registers(self.gripper_busy_addr, 1,
                                                  unit=self.rg2ft_device_addr)) == 1:
            continue

        self.is_writing_registers = False
        #grip executed successfully
        return True

    '''
    Init function, is a constructor, called each time a new instance of 
    this object is created Establishes contact with the registers through modbus 
    interface - prints an error message if not successfull
    Creates a ROS node
    Creates ROS publishers and subscribers
    
    starts the run() method at the end of init to start working
    
    Parameters
        self
    Returns
        None
    Throws
        None
    '''
    def __init__(self):
        self.wanted_hz = 20
        self.current_force = 3
        self.cmd_string = None
        self.has_stored_cmd = False

        box_ip = "192.168.1.1"  # OnRobot computebox IP address
        # Creates a client in this program
        self.client = ModbusClient(box_ip, port=502)
        # that uses the box ip and port 502 (as per standard for modbus)
        return_val = self.client.connect()  # Tries to connect to the computebox
        # prints in console if a connection is established

        print("Established connection to compute box?: ", return_val)
        #save the last retrived data to send incase read failed this time
        self.last_current_width = -1
        self.last_prox_L = -1
        self.last_prox_R = -1
        self.last_F_x_R = -1
        self.last_F_x_L = -1
        self.last_F_y_R = -1
        self.last_F_y_L = -1
        self.last_F_z_R = -1
        self.last_F_z_L = -1

        self.write_cmd_prio = False

        self.is_reading_registers = False
        self.is_writing_registers = False
        if return_val:
            # init publishers
            #rospy = rospy
            rospy.init_node('publisher',disable_signals=True)
            self.pub_proximity = rospy.Publisher('/gripper_interface/proximity_data/',
                                                 String, queue_size=1)
            self.pub_force = rospy.Publisher('/gripper_interface/force_torque_data/',
                                             String, queue_size=1)
            self.pub_misc = rospy.Publisher('/gripper_interface/misc_data',
                                            String, queue_size=1)
            self.pub_width = rospy.Publisher('/gripper_interface/gripper_width',
                                             String, queue_size=1)
            #rospy.init_node('topic_subscriber')
            self.sub_gripper_cmd = rospy.Subscriber('gripper_interface/gripper_cmd/',
                                                    String ,self.gripper_cmd_handler,
                                                    queue_size=1)
            publishing_rate_Hz = rospy.Rate(1)

            #dubblecheck that offset values are right
            self.client.write_register(self.proximity_offset_L_addr, 190,
                                       unit=self.rg2ft_device_addr)
            self.client.write_register(self.proximity_offset_R_addr, 260,
                                       unit=self.rg2ft_device_addr)

        else:
            print("Catastrophic error please check yo self!!")


if __name__ == '__main__':
    #starts script here when executing this python file,
    # only used when debugging the code

    C = Rg2ftModbusROSInterface()
    C.run()


