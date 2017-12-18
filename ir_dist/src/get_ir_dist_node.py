#!/usr/bin/env python

import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped
from duckietown_msgs.srv import SetParam
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import numpy as np

samples_per_message = 4
publish_rate = 30  # in hz

class lane_controll_node:
	def __init__(self):
		# set up publisher for wheel commands
		self.measurement_queue = list()
		self.pub = rospy.Publisher("~ir_dist", Range, queue_size=1)
		rospy.Timer(rospy.Duration(1.0/publish_rate), self.publish_dist, oneshot=False)
		rospy.Timer(rospy.Duration(1.0/(samples_per_message * publish_rate)), self.take_sample)

	def publish_dist(self, time):
		if len(self.measurement_queue) > 0:
			msg = Range()
			msg.range = np.asarray(self.measurement_queue).mean()
			self.measurement_queue = list()
			self.pub.publish(msg)

	def take_sample(self, time):
		voltage = readadc(0, SPICLK, SPIMOSI, SPIMISO, SPICS) * (3.3 / 1024)
		dist = 37.853 * voltage ** (-0.878)
		self.measurement_queue.append(dist)


def map_to(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspin, True)

        adcout >>= 1       # first bit is 'null' so drop it
        return adcout

# change these as desired - they're the pins connected from the
# SPI port on the ADC to the Cobbler
SPICLK = 18
SPIMISO = 23
SPIMOSI = 24
SPICS = 25

# set up the SPI interface pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

if __name__ == "__main__":
	rospy.init_node("get_ir_dist_node", anonymous=False)
        rospy.loginfo("starting ir dist node")
	lane_controll_node()
	rospy.spin()

