#!/usr/bin/env python3
import rospy
import uuid

from vpa_basecontrol.msg import WheelsCmd,WheelEncoderStamped
from include.wheel_encoder import WheelEncoderDriver, WheelDirection
from std_msgs.msg import Header
from math import pi
class WheelEncoderNode():
    """Node handling a single wheel encoder.

    This node is responsible for reading data off of a single wheel encoders.
    Robots with N wheels will need to spin N instances of this node.
    This node is compatible with any rotary encoder that signals ticks as rising edges
    on a digital GPIO pin.

    Though there are two encoders output from the motor, the DB18 decides to use only one of it.
    As a result, the direction must be implied from the command. This may be problematic in many cases

    """

    def __init__(self) -> None:
        # param from the launch file
        self._veh = rospy.get_param(param_name="~veh",default="")
        self._name = rospy.get_param(param_name="~name",default="left")
        self._gpio_pin = rospy.get_param(param_name="~gpio",default=18)
        self._resolution = rospy.get_param(param_name="~resolution",default=135)
        self._configuration = rospy.get_param(param_name="~configuration",default="left")
        self._publish_frequency = rospy.get_param(param_name="~publish_frequency",default=30)

        calib_file = "include/encoder_config/" +  self._name + "_wheel.yaml"

        # try:
        #     with open(calib_file,"r") as f:
        #         calib_data = yaml.safe_load(f)
        #     custom_resolution = int(calib_data["resolution"])
        #     rospy.set_param("~resolution", custom_resolution)
        #     self._resolution = custom_resolution
        #     # This seems unnecessary for now
        # except FileNotFoundError:
        #     rospy.logwarn(f"No custom encoder calibration found at: {calib_file}. " "Using default parameters.")
        # except:
        #     rospy.logwarn("Other errors")

        #tick storage
        self._tick = 0
        self._tick_last = 0
        self._last_tick_timing = 0
        self._omega = 0
        self._omega_last = 0

        self._tick_pub = rospy.Publisher(
            "~tick", WheelEncoderStamped, queue_size=1
        )
        #
        self._driver = WheelEncoderDriver(self._gpio_pin, self._encoder_tick_cb)
        # from this topic, we guess which direction the wheel is spinning
        self.sub_topic = rospy.Subscriber("wheels_cmd", WheelsCmd, self._wheels_cmd_cb, queue_size=1)
        # publish the wheel speed
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._publish_frequency), self._cb_publish)

    def _wheels_cmd_cb(self, msg):
        if self._configuration == "left":
            if msg.vel_left >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)
        elif self._configuration == "right":
            if msg.vel_right >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)

    def _encoder_tick_cb(self, tick_no):
        """
        Callback that receives new ticks from the encoder.

            Args:
                tick_no (int): cumulative total number of ticks
        """
    
        self._tick = tick_no
        
        now  = rospy.get_time()
        diff = now - self._last_tick_timing
        self._last_tick_timing = now
        self._omega = (2*pi/self._resolution)/diff
        if self._omega > 25:
            self._omega =self._omega_last
        
        # if no tick changes for a while -> set to 0

    
    def _frequency_change_cb(self):
        """
        Callback triggered when the publish frequency changes.
        """
        self._timer.shutdown()
        frequency = self._publish_frequency.value
        self._timer = rospy.Timer(rospy.Duration(1.0 / frequency), self._cb_publish)
        rospy.loginfo(f"Publish frequency now set to {frequency}Hz")
    
    def _cb_publish(self, _):
        delta_tick = self._tick - self._tick_last
        self._tick_last = self._tick
        
<<<<<<< HEAD
=======
        if self._omega > 15 * self._omega_last:
            self._omega = self._omega_last
        
>>>>>>> 2a4d1e5f7f2a24cb01ed6fe26cdbdae103058532
        if delta_tick == 0:
            _omega = 0
        else:
            _omega = self._omega
<<<<<<< HEAD
        
        self._omega_last = _omega
        
=======
        self._omega_last = _omega
>>>>>>> 2a4d1e5f7f2a24cb01ed6fe26cdbdae103058532
        header = Header()
        header.frame_id = f"{self._veh}/{self._name}_wheel_axis"
        header.stamp = rospy.Time.now()
        self._tick_pub.publish(
            WheelEncoderStamped(
                header=header,
                data=self._tick,
                resolution=self._resolution,
                type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
                omega=_omega
            )
        )

if __name__ == "__main__":
    rand = str(uuid.uuid4())[:8]
    rospy.init_node("wheel_encoder_%s" % (rand,))
    node = WheelEncoderNode()
    rospy.spin()