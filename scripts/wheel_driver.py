#!/usr/bin/env python

import rospy
from dagu_wheels_driver import DaguWheelsDriver

from std_msgs.msg import Bool
from vpa_db19interface.msg import WheelsCmd,WheelEncoderStamped
from vpa_db19interface.cfg import omegaConfig
from dynamic_reconfigure.server import Server
class WheelsDriverNode():
    """Node handling the motor velocities communication.


    Subscribers:
       ~wheel_cmd (:obj:`self.defined`)
    """

    def __init__(self):
        
        self.estop = True

        # Setup the driver
        self.driver = DaguWheelsDriver()
        #self.log("Initialized.")
        

        
        self._left_omega      = 0
        self._right_omega     = 0
        self._vel_left_ref    = 0
        self._vel_right_ref   = 0
        
        self._vel_left        = 0
        self._vel_right       = 0
        
        self._err_int         = 0
        
        self.kp = 0.08
        self.ki = 0.0003
        
        # Subscribers
        self.sub_topic = rospy.Subscriber("wheels_cmd", WheelsCmd, self.wheels_cmd_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("/global_start", Bool, self.estop_cb, queue_size=1)
        self.sub_left_tick  = rospy.Subscriber("left_wheel_encoder_node/tick",WheelEncoderStamped,self.left_omega_callback,queue_size=1)
        self.sub_right_tick = rospy.Subscriber("right_wheel_encoder_node/tick",WheelEncoderStamped,self.right_omega_callback,queue_size=1)
        self.srv = Server(omegaConfig,self.dynamic_reconfigure_callback)
        rospy.loginfo("Wheel Driver Initialized")
        
    def wheels_cmd_cb(self, msg:WheelsCmd):
        """
        Callback that sets wheels' speeds.
        """
        if self.estop:
            self.driver.set_wheels_speed(left=0, right=0)
        else:
            self._vel_left_ref    = msg.vel_left
            self._vel_right_ref   = msg.vel_right

            # vel_left    = self.omega_pid(self._left_omega,vel_left_ref)
            # vel_right   = self.omega_pid(self._right_omega,vel_right_ref)
        
        
        # it is likely we also do the latency check here, but for now it is ignored
    def dynamic_reconfigure_callback(self,config,level):
        self.kp = config.kp
        self.ki = config.ki
        return config
        
    def estop_cb(self,msg):
        """
        Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """
        self.estop = msg.data
        if self.estop:
            rospy.log("Global Brake Activated")
        else:
            rospy.log("Global Brake Released")

    def on_shutdown(self):
        """
        Shutdown procedure.

        Publishes a zero velocity command at shutdown.
        """
        self.driver.set_wheels_speed(left=0.0, right=0.0)
    
    def left_omega_callback(self,tick_msg:WheelEncoderStamped):
        self._left_omega    = tick_msg.omega
        self._vel_left      = self.omega_pid(self._left_omega,self._vel_left_ref)
        self.driver.set_wheels_speed(left=self._vel_left, right=self._vel_right)
        
        
    def right_omega_callback(self,tick_msg:WheelEncoderStamped):
        self._right_omega = tick_msg.omega
        self._vel_right   = self.omega_pid(self._right_omega,self._vel_right_ref)
        self.driver.set_wheels_speed(left=self._vel_left, right=self._vel_right)

    def omega_pid(self,cur_omega,cur_ref_omega):
        if cur_ref_omega < 0.01:
            return 0
        kp = self.kp
        ki = self.ki
        err_omega = cur_ref_omega - cur_omega
        self._err_int += err_omega
        
        #print('current_err',err_omega)
        throttle_control = kp * err_omega + ki * self._err_int
        if throttle_control > 1:
            throttle_control = 1
        if throttle_control < -1:
            throttle_control = -1
        return throttle_control
    
if __name__ == "__main__":
    rospy.init_node("wheel_driver")
    node = WheelsDriverNode()
    rospy.spin()