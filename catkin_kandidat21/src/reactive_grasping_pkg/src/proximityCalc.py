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
import armCalc
from forceCalc import ForceCalcClass


'''
Class with helper function to handle proximity logic when starting a grasp sequence
'''
class ProximityCalcClass:
    '''
    Reads data from a topic, saves it internally for later use

    Paramters
        self
        msg - data that has been published on a topic and now being read
    Returns
        None
    Throws
        None
    '''
    def prox_data_handler(self, msg):
        self.current_msg = msg.data

    '''
        Reads data from a topic, saves it internally for later use

        Paramters
            self
            msg - data that has been published on a topic and now being read
        Returns
            None
        Throws
            None
        '''
    def width_data_handler(self, wmsg):
        self.current_wmsg = wmsg.data

    '''
        Reads data from a topic, saves it internally for later use

        Paramters
            self
            
        Returns
            width/10 - current gripper width in mm
        Throws
            None
        '''
    def get_width(self):
        width = self.current_wmsg
        return int(width)


    '''
    Splices left proximity value from the msg that was saved earlier
    
    Parameters
        self
    Returns
        proxL - left proximity value
    Throws
        None
    '''
    def get_prox_L(self):
        proxL = self.current_msg.split('=', )[2].split(']')[0]


        try:
            return float(proxL)
        except:
            print('EXCEPTION', proxL)
            return 0

    '''
        Splices right proximity value from the msg that was saved earlier

        Parameters
            self
        Returns
            proxR - right proximity value
        Throws
            None
        '''
    def get_prox_R(self):
        proxR = self.current_msg.split('=', )[1].split(',')[0]
   
        try:
            return float(proxR)
        except TypeError:
            print('EXCEPTION', proxR)

        return float(proxR)

    def prox_check_force_condition(self, tol_force_N):
        # rospy.init_node('topic_subscriber_proxcalc')

        arm = armCalc.UR10_robot_arm()
        forceLogic = ForceCalcClass()



        pub_cmd_proximitycalc = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)
        # rospy.init_node('talkerdffdf', anonymous=True)
        # Run gripping command from main? or just return a "good" status, start closing grippers and then run proxCheck()
        # again once fingers are closer to object? interesting if we could with this grip a moving object?
        if math.isclose(self.get_prox_L(), self.get_prox_R(), abs_tol=self.tolerance_mm):
            print('FORCE L R : ', forceLogic.get_F_z_L()/-10, forceLogic.get_F_z_R()/10, ' real values: ', forceLogic.get_F_z_L(), forceLogic.get_F_z_R())
            if abs(forceLogic.get_F_z_R()/10) > tol_force_N and abs(forceLogic.get_F_z_L()/10) > tol_force_N:
                return True, print('Yay!', self.get_prox_L(), self.get_prox_R())  # ???
            else:
                print('L', self.get_prox_L(), ' R', self.get_prox_R(), 'FzR', forceLogic.get_F_z_R(), 'FzL', forceLogic.get_F_z_L())

                while not rospy.is_shutdown():
                    connections = pub_cmd_proximitycalc.get_num_connections()
                    print('watiing for connecting in prox check force')
                    if connections > 0:
                        # target_width = 100 - self.get_prox_R() - self.get_prox_L() - self.tolerance_mm
                        # if target_width < 0:
                        #    target_width = 2
                        old_width = int(self.get_width()/10)
                        step = 2

                        pub_cmd_proximitycalc.publish('operate_gripper_step_width(' + str(step) + ')')
                        print(self.get_width()/10)


                        arm.depth_compensation_gripper((self.get_width()/10), (self.get_width()/10) - step)

                        while not math.isclose(self.get_width()/10, old_width - step, abs_tol= 1):
                            continue

                        break
                    else:
                        time.sleep(0.1)

        # If distance from finger L + tolerance is larger than the distance from finger R, this mean we need to move
        #    the arm in the direction of finger R. Standing on the "robots side", the finger marked as R is on the left,
        # sooo this means we need to decide from what perspective we are counting from.
        elif (self.get_prox_L() - self.tolerance_mm) > self.get_prox_R():
            # Move gripper towards finger R by tiny amount depending on Hz? SLOWLY! send this to armCalc
            print('1')
            arm.move_gripper_L(2)
            time.sleep(0.75)


        elif (self.get_prox_R() - self.tolerance_mm) > self.get_prox_L():
            # Same as above but inverted send this to armCalc
            print('2')
            arm.move_gripper_R(2)
            time.sleep(0.75)


 #  Check if Proximity readings match within tolerance, if not adjust position
    # TODO This needs to access ROS node for controlling the arm to adjust position
    def prox_check(self, tolerance, solid):
        #rospy.init_node('topic_subscriber_proxcalc')


        arm = armCalc.UR10_robot_arm()
        forceLogic = ForceCalcClass()

        pub_cmd_proximitycalc = rospy.Publisher('/gripper_interface/gripper_cmd/', String, queue_size=10)
        #rospy.init_node('talkerdffdf', anonymous=True)
        # Run gripping command from main? or just return a "good" status, start closing grippers and then run proxCheck()
        # again once fingers are closer to object? interesting if we could with this grip a moving object?
        if math.isclose(self.get_prox_L(), self.get_prox_R(), abs_tol=self.tolerance_mm):
            if solid:
                while not rospy.is_shutdown():
                    connections = pub_cmd_proximitycalc.get_num_connections()
                    if connections > 0:
                        # target_width = 100 - self.get_prox_R() - self.get_prox_L() - self.tolerance_mm
                        # if target_width < 0:
                        #    target_width = 2
                        step = 2
                        pub_cmd_proximitycalc.publish('operate_gripper(' + str(int(self.get_width()) -step) +', 40)')
                        print(self.get_width())
                        arm.depth_compensation_gripper(self.get_width(), int(self.get_width()) - step)
                        time.sleep(0.5)
                        break
                    else:
                        time.sleep(0.1)
                    if forceLogic.get_F_z_L() > 10 and forceLogic.get_F_z_R() >10:
                       return True, print('solid grip!')

            if math.isclose(self.get_prox_L(), 0, abs_tol=tolerance) and math.isclose(self.get_prox_R(), 0,
                                                                                       abs_tol=tolerance):
                return True, print('Yay!', self.get_prox_L(), self.get_prox_R())  # ???
            else:
                print('L', self.get_prox_L(), ' R', self.get_prox_R())

                while not rospy.is_shutdown():
                    connections = pub_cmd_proximitycalc.get_num_connections()
                    if connections > 0:
                        #target_width = 100 - self.get_prox_R() - self.get_prox_L() - self.tolerance_mm
                        #if target_width < 0:
                        #    target_width = 2
                        step = 2
                        pub_cmd_proximitycalc.publish('operate_gripper_step_width(' + str(step)+')')
                        print(self.get_width())
                        arm.depth_compensation_gripper((self.get_width()), (self.get_width()) - step)
                        time.sleep(0.5)
                        break
                    else:
                        time.sleep(0.1)

        # If distance from finger L + tolerance is larger than the distance from finger R, this mean we need to move
    #    the arm in the direction of finger R. Standing on the "robots side", the finger marked as R is on the left,
        # sooo this means we need to decide from what perspective we are counting from.
        elif (self.get_prox_L() - self.tolerance_mm)> self.get_prox_R():
            # Move gripper towards finger R by tiny amount depending on Hz? SLOWLY! send this to armCalc
            print('1')
            arm.move_gripper_L(2)
            time.sleep(0.75)


        elif (self.get_prox_R() - self.tolerance_mm) > self.get_prox_L():
            # Same as above but inverted send this to armCalc
            print('2')
            arm.move_gripper_R(2)
            time.sleep(0.75)

    '''
    Constructor, called each time a new instance of this class is created
    Sets internal variables
    Creates publishers and subscribers
    sets internal tolerance
    Parameters
        self
    Returns
        None
    Throws
        None
    '''
    def __init__(self):
        self.current_msg = '[proximity_value_R=300,proximity_value_L=550]'
        self.tolerance_mm = 40
        self.proximity_sub = rospy.Subscriber('/gripper_interface/proximity_data/', String, self.prox_data_handler)
        self.width_sub = rospy.Subscriber('/gripper_interface/gripper_width', String, self.width_data_handler)

def main():

    None


if __name__ == '__main__':
    main()
