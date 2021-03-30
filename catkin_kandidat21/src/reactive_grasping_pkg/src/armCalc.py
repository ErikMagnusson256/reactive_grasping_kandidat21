#!/usr/bin/env python
#TODO add description of file

import rospy
import std_msgs.msg
from std_msgs.msg import String
import time
#from robot_ctrl.clik import Clik
import rospy, rospkg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from robot_interface.msg import RobotState, EndEffectorState
from robot_ctrl.msg import CtrlTarget
import numpy as np


class UR10InterfaceNode:
    """
            ROS Parameters
            ----------
            {ns}/robot_interface/update_rate : int
                Update rate of the controller
            {ns}/robot_interface/mode : str
                Control mode for robot ['urscript', 'integrator', 'gazebo']
            {ns}/robot_interface/acc_max : float
                Maximum acceleration of controller
            {ns}/robot_interface/joint_ctrl : bool
                Switch for joint/end-effector control. True => Joint space control
            {ns}/robot_interface/init_pos : list
                Initial joint position for simulation (used if mode is 'integrator' or 'gazebo')
            Attributes
            ----------
            n : int
                Number of joints
            joint_names : list
                Joint names in robot URDF description
            max_msg_deadtime : float
                Threshold (in seconds) of how old command can be in order to update controller
            rate: rospy.Rate
                ROS update rate of controller
            dt: float
                Sample time of controller
            mode: RobotMode
                Robot Mode [URSCRIPT, INTEGRATOR, GAZEBO]
            joint_ctrl: bool
                Switch for joint/end-effector control. True => Joint space control
            a_max: float
                Maximum acceleration of controller
            init_pos: list
                Initial joint position for simulation (used if mode is 'integrator' or 'gazebo')
            state_stamp : rospy.Time
                Time stamp of last updated robot state
            q: list
                List of robot joint positions
            dq: list
                List of robot joint velocities
            cmd_stamp : rospy.Time
                Time stamp of last received command message
            pos_cmd: list
                List of target positions
            vel_cmd: list
                List of target velocities
            Methods
            -------

            """

    def __init__(self):
        self.n = 6
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.max_msg_deadtime = 0.1
        self.rate = None
        self.dt = None
        self.mode = None
        self.joint_ctrl = None
        self.a_max = None
        self.init_pos = None
        self.clik_ctrl = None

        self.state_stamp = None
        self.q = None
        self.dq = None
        self.cmd_stamp = None
        self.pos_cmd = None
        self.vel_cmd = None

        if not self.load_params():
            return

        rospy.Subscriber("robot_interface/vel_command", CtrlTarget, self.vel_command_update)
        rospy.Subscriber("robot_interface/pos_command", CtrlTarget, self.pos_command_update)
        self.joint_state_publisher = rospy.Publisher("robot_interface/joint_states", JointState, queue_size=1)
        self.robot_state_publisher = rospy.Publisher("robot_interface/state", CtrlTarget, queue_size=1)

        if self.mode == RobotMode.URSCRIPT:
            rospy.loginfo('Starting UR10 interface')
            self.urscript_publisher = rospy.Publisher("ur_hardware_interface/script_command", String, queue_size=1)
            rospy.Subscriber("joint_states", JointState, self.joint_state_update)


        # Go into spin
        self.spin()

    def vel_command_update(self, msg):
        """Callback for ROS topic {ns}/robot_interface/vel_command to update velocity command.
        Parameters
        ----------
        msg : robot_ctrl/CtrlTarget
            CtrlTarget message with velocity information of command.
        """
        if (self.joint_ctrl and len(msg.velocity) == self.n) or (not self.joint_ctrl and len(msg.velocity) == 6):
            self.cmd_stamp = rospy.Time.now()
            self.vel_cmd = np.array(msg.velocity)
            self.pos_cmd = None


    def pos_command_update(self, msg):
        """Callback for ROS topic {ns}/robot_interface/pos_command to update position command.
        Parameters
        ----------
        msg : robot_ctrl/CtrlTarget
            CtrlTarget message with position information of command.
        """
        if (self.joint_ctrl and len(msg.position) == self.n) or (not self.joint_ctrl and len(msg.position) == 7):
            self.cmd_stamp = rospy.Time.now()
            self.vel_cmd = None
            self.pos_cmd = np.array(msg.position)

    def joint_state_update(self, msg):
        """Callback for ROS topic {ns}/joint_states to update joint state. (Used in URSCRIPT mode)
        Parameters
        ----------
        msg : JointState
            ROS JointState message with joint state information.
        """
        q = np.zeros(self.n)
        dq = np.zeros(self.n)
        for i in range(self.n):
            q[i] = msg.position[msg.name.index(self.joint_names[i])]
            dq[i] = msg.velocity[msg.name.index(self.joint_names[i])]
        self.q = q
        self.dq = dq
        self.state_stamp = msg.header.stamp

    def publish_position_command(self):
        """Send position command
        URSCRIPT mode: Sends command to robot
        INTEGRATOR mode: Set simulated robot in target position
        """
            if self.joint_ctrl:
                urscript_cmd = 'movej'
                or_cmd = self.pos_cmd[3:]
            else:
                urscript_cmd = 'movel'
                quat = np.array(self.pos_cmd[3:])
                quat = quat / np.linalg.norm(quat)
                angle = 2 * math.acos(quat[0])
                s = math.sqrt(1 - quat[0] * quat[0])
                or_cmd = [angle, 0, 0] if s < 1e-3 else (angle / s * quat[1:]).toList()
            command = urscript_cmd + "([" + str(self.pos_cmd[0]) + "," + str(self.pos_cmd[1]) + "," \
                      + str(self.pos_cmd[2]) + "," + str(or_cmd[0]) + "," + str(or_cmd[1]) + "," \
                      + str(or_cmd[2]) + "])"
            self.urscript_publisher.publish(command)


    def publish_velocity_command(self):
        """Send velocity command
        URSCRIPT mode: Sends command to robot
        """
            urscript_cmd = 'speedj' if self.joint_ctrl else 'speedl'
            command = urscript_cmd + "([" + str(self.vel_cmd[0]) + "," + str(self.vel_cmd[1]) + "," \
                      + str(self.vel_cmd[2]) + "," + str(self.vel_cmd[3]) + "," + str(self.vel_cmd[4]) + "," \
                      + str(self.vel_cmd[5]) + "], " + str(self.a_max) + " , " + str(2 * self.dt) + ")"
            self.urscript_publisher.publish(command)



    def publish_robot_state(self):
        """Publishes current joint state. """
        joint_state = JointState()
        joint_state.header.stamp = self.state_stamp
        joint_state.header.frame_id = ''
        joint_state.name = self.joint_names
        joint_state.position = self.q
        joint_state.velocity = self.dq
        joint_state.effort = []
        self.joint_state_publisher.publish(joint_state)

        msg = CtrlTarget()
        msg.header.stamp = self.state_stamp
        if self.joint_ctrl:
            msg.velocity = self.dq
            msg.position = self.q
        else:
            [ee_pos, ee_quat] = self.clik_ctrl.forward_kinematics(self.q.tolist())
            msg.position = ee_pos + ee_quat
            jac = np.array(self.clik_ctrl.jacobian(self.q.tolist()))
            msg.velocity = jac.dot(self.dq)
        self.robot_state_publisher.publish(msg)

    def load_params(self):
        """Loads parameters from ROS parameter server.
        Returns
        -------
        bool
            Returns True if all mandatory parameters were available at the ROS parameter server.
            Returns False otherwise.
        """
        # Get joint_ctrl
        if not rospy.has_param('robot_interface/joint_ctrl'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_interface/joint_ctrl not set.")
            return False
        self.joint_ctrl = rospy.get_param('robot_interface/joint_ctrl')
        # Get update rate
        if not rospy.has_param('robot_interface/update_rate'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_interface/update_rate not set.")
            return False
        update_rate = rospy.get_param('robot_interface/update_rate')
        self.rate = rospy.Rate(update_rate)
        self.dt = 1. / update_rate
        # Get max acceleration
        if not rospy.has_param('robot_interface/acc_max'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_interface/acc_max not set.")
            return False
        self.a_max = rospy.get_param('robot_interface/acc_max')


        rospy.loginfo("Loaded robot_interface parameters.")
        return True

    def spin(self):
        while (not rospy.is_shutdown()):
            # Publish command
            if self.cmd_stamp is not None:
                command_msg_deadtime = (rospy.Time.now() - self.cmd_stamp).to_sec()
                if command_msg_deadtime < self.max_msg_deadtime:
                    if self.vel_cmd is not None:
                        self.publish_velocity_command()
                    if self.pos_cmd is not None:
                        self.publish_position_command()
                elif self.mode is RobotMode.INTEGRATOR:
                    self.dq = np.zeros(self.n)


            # Publish state
            if self.state_stamp is not None:
                state_msg_deadtime = (rospy.Time.now() - self.state_stamp).to_sec()
                if state_msg_deadtime < self.max_msg_deadtime:
                    self.publish_robot_state()

            # Sleep
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # Init node
        rospy.init_node('UR10InterfaceNode')

        # Initializing and continue running the main class
        UR10InterfaceNode()

    except rospy.ROSInterruptException:
        pass

#############################################################
# only an example of easy implement for trial
def move_R(distance):
    # send moveJ base joint tiny tiny amount to right for starters?
    # do the same for move_L()
    None

#TODO descriptopm
#essentially this is the interface we can use to controll the arm easily by eg. armCalc.UR10_robot_arm.move_start_pos()
class UR10_robot_arm:

    # For now, move_gripper_R will move the arm in the direction of the finger marked R
    def move_gripper_R(self, distance_mm):
        # TODO to be able to calculate the moveL command we need to know orientation of gripper, put thought into
        # what joints are interesting to determine this. All probably? by calculating all joints we should be able to
        # know the direction the gripper is pointing, and then we read the hand position from the arm. Is there a
        # built in function for this?
        # By reading the orientation of the last joint we can turn that orthogonal plane into a line we can move along.
        # This way we would ensure that we always move in the direction we intend.
        # This would need to be divided into separate functions to make it tidy and functional.
        None

    def move_gripper_L(self, distance_mm):
        # same as above but in the other direction along calculated line.
        None

        #its possible we want to
    def move_gripper_forwards(self, distance_mm):
        # "only" need to move in the direction of the calculated orientation.
        None

    def move_gripper_backwards(self, distance_mm):
        # "only" need to move against the direction of the calculated orientation.
        None

    def move_gripper_up(self, distance_mm):
        # move vertically, or move "up" from the grippers perspective? keep gripper at same direction?
        None

    def move_gripper_down(self, distance_mm):
        # same as up function
        None

    # do we want functions that move the arm in a "absolute" direction? what is described above is all relative to the
    # gripper. These are needed to adjust position from the sensor feedback, but might be unnecessarily complicated
    # when moving larger distances or to rough positions for pickup/drop off.

    # moves arm to a known starting position with a known pose
    def move_to_starting_pos(self):
        None

    # init internal variables and start communication with the arm itself
    # some type of check to see if communication with arm is successfull
    def __init__(self):
        None

if __name__ == '__main__':
    None