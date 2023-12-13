#!/usr/bin/env python3

import rospy

from vpa_basecontrol.msg import WheelsCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
class KinematicsNode():
    """
    The `KinematicsNode` maps car commands send from various nodes to wheel commands that the robot can execute.
    `KinematicsNode` utilises the car geometry as well as a number of tunining and limiting
    parameters to calculate the wheel commands that the wheels should execute in order for the
    robot to perform the desired car commands (inverse kinematics).    

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired velocity, default is 1.0
        ~trim (:obj:`float`): trimming factor that is typically used to offset differences in the
            behaviour of the left and right motors, it is recommended to use a value that results in
            the robot moving in a straight line when forward command is given, default is 0.0
        ~baseline (:obj:`float`): the distance between the two wheels of the robot, default is 0.1
        ~radius (:obj:`float`): radius of the wheel, default is 0.0318
        ~k (:obj:`float`): motor constant, assumed equal for both motors, default is 27.0
        ~limit (:obj:`float`): limits the final commands sent to the motors, default is 1.0
        ~v_max (:obj:`float`): limits the input velocity, default is 1.0
        ~omega_max (:obj:`float`): limits the input steering angle, default is 8.0

    """

    def __init__(self) -> None:
        
        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        # Read parameters from a robot-specific yaml file if such exists
        # self.read_params_from_calibration_file()

        self._k = rospy.get_param("~k")
        self.estop = True
        # may apply dynamic-reconfiguration to the following parameters
        self._gain      = rospy.get_param("~gain")
        self._trim      = rospy.get_param("~trim")
        self._limit     = rospy.get_param("~limit")
        self._baseline  = rospy.get_param("~baseline") 
        self._radius    = rospy.get_param("~radius")
        self._v_max     = rospy.get_param("~v_max")
        self._omega_max = rospy.get_param("~omega_max")

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher(
            "wheels_cmd", WheelsCmd, queue_size=1
        )    
        # Setup subscribers
        self.sub_car_cmd = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_callback)
        # ---
        self.sub_e_stop = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        rospy.loginfo("Kinematics Node Initialized, Standing by for twist command")

    def car_cmd_callback(self, msg_car_cmd:Twist):

        #msg_car_cmd.linear.x    = self.trim(msg_car_cmd.linear.x, low=-self._v_max, high=self._v_max)
        #msg_car_cmd.angular.z   = self.trim(msg_car_cmd.angular.z, low=-self._omega_max, high=self._omega_max)

        msg_car_cmd.linear.x    = max(min(msg_car_cmd.linear.x,self._v_max),-self._v_max)
        msg_car_cmd.angular.z   = max(min(msg_car_cmd.angular.z,self._omega_max),-self._omega_max)

        # assuming same motor constants k for both motors
        k_r = k_l = self._k

        # adjusting k by gain and trim
        k_r_inv = (self._gain + self._trim) / k_r
        k_l_inv = (self._gain - self._trim) / k_l

        #
        omega_r = (msg_car_cmd.linear.x + 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius
        omega_l = (msg_car_cmd.linear.x - 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius

        # u_r = omega_r * k_r_inv
        # u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        # u_r_limited = self.trim(u_r, -self._limit, self._limit)
        # u_l_limited = self.trim(u_l, -self._limit, self._limit)
        # u_r_limited = max(min(u_r,self._limit),-self._limit)
        # u_l_limited = max(min(u_l,self._limit),-self._limit)

        msg_wheels_cmd              = WheelsCmd()
        msg_wheels_cmd.vel_right    = omega_r
        msg_wheels_cmd.vel_left     = omega_l
        
        if self.estop:
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left  = 0
        
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    # def trim(value,low,high):
    #     return max(min(value, high), low)
    def estop_cb(self,msg):
        """
        Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """
        self.estop = msg.data
        if self.estop:
            rospy.loginfo("Global Brake Activated")
        else:
            rospy.loginfo("Global Brake Released")
            
if __name__ == "__main__":
    rospy.init_node("kinematics_node")
    node = KinematicsNode()
    rospy.spin()