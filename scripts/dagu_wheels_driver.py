#!/usr/bin/env python

from math import fabs, floor

from include.hat_driver.motor import MotorDirection
# from dt_robot_utils import get_robot_configuration

from include.hat_driver.hat import HATv2

# M_value = MotorDirection()


class DaguWheelsDriver:
    """Class handling communication with motors."""

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level

    def __init__(self):
        #rcfg = get_robot_configuration()
        self.hat = HATv2()
        self.leftMotor = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")
        # print out some stats
        # this = self.__class__.__name__
        # print(f"[{this}] Running in configuration `{rcfg.name}`, using driver `{DTHAT.__name__}`")
        # print(f"[{this}] Motor #1: {self.leftMotor}")
        # print(f"[{this}] Motor #2: {self.rightMotor}")
        # initialize state
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self._pwm_update()
        # hardware testing flag
        self._is_performing_test = False
    
    def start_hardware_test(self):
        self._is_performing_test = True
    
    def finish_hardware_test(self):
        self._is_performing_test = False

    def set_wheels_speed(self, left: float, right: float, is_test_cmd: bool = False):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
           is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test

        """
        if self._is_performing_test and is_test_cmd or not self._is_performing_test:
            self.leftSpeed = left
            self.rightSpeed = right
            self._pwm_update()

    def _pwm_value(self, v, min_pwm, max_pwm):
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def _pwm_update(self):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.
        """
        vl = self.leftSpeed
        vr = self.rightSpeed

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode = 0
        rightMotorMode = 0

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = MotorDirection.BACKWARD

        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)

    def __del__(self):
        """Destructor method.

        Releases the motors and deletes tho object.
        """
        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat
