# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# Authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#          Shih-Yuan Liu <syliu@mit.edu>
# Modified by PCH 2017

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DaguWheelsDriver:
    LEFT_MOTOR_MIN_PWM = 60        # Minimum PWM for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum PWM for left motor  
    RIGHT_MOTOR_MIN_PWM = 60       # Minimum PWM for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum PWM for right motor  
    SPEED_TOLERANCE = 1.e-2        # Speed tolerance level

    def __init__(self):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.right_motor = self.motorhat.getMotor(2)
        self.left_motor = self.motorhat.getMotor(1)

        self.u_r = 0.0 # speed of right wheel [-1.0, 1.0]
        self.u_l = 0.0 # speed of left wheel [-1.0, 1.0]
        self.update_motors()

    def convert_to_pwm(self, u, min_pwm, max_pwm):
        """
        Convert from wheel speed to pulse-width-modulation value."
        """
        pwm = 0
        if fabs(u) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(u) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def update_motors(self):
        """
        Update the car motors based on the current wheel speed targets.
        """
        vr = self.u_r
        vl = self.u_l

        pwmr = self.convert_to_pwm(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        pwml = self.convert_to_pwm(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)

        if fabs(vr) < self.SPEED_TOLERANCE:
            right_motor_mode = Adafruit_MotorHAT.RELEASE
            pwmr = 0
        elif vr > 0:
            right_motor_mode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            right_motor_mode = Adafruit_MotorHAT.BACKWARD

        if fabs(vl) < self.SPEED_TOLERANCE:
            left_motor_mode = Adafruit_MotorHAT.RELEASE
            pwml = 0
        elif vl > 0:
            left_motor_mode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            left_motor_mode = Adafruit_MotorHAT.BACKWARD

        self.right_motor.setSpeed(pwmr)
        self.right_motor.run(right_motor_mode)
        self.left_motor.setSpeed(pwml)
        self.left_motor.run(left_motor_mode)

    def set_wheel_speeds(self, right, left):
        """
        Specify target wheel speeds.
        """
        self.u_r = right
        self.u_l = left
        self.update_motors()

    def __del__(self):
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat
