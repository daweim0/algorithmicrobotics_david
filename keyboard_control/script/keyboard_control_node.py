#!/usr/bin/env python

# Author: PCH 2017

"""
Description: Publish car velocity commands based on keyboard input.
   '-'/'+' to reduce/increase the forward velocity of the car (when moving)
   '['/']' to reduce/increase the angular velocity of the car (when turning)
    UP_ARROW to drive straight forward
    DOWN_ARROW to stop
    LEFT_ARROW to turn left (note: does not clear forward velocity)
    RIGHT_ARROW to turn right (note: does not clear forward velocity)
    Ctrl-C to quit
"""

# TODO: include rospy
# TODO: include any msgs you use
import sys, termios

class KeyboardControl:
    def __init__(self):
        self.node_name = "TODO"

        # Speed settings
        self.lin_vel = 0.38
        self.ang_vel = 1.0
        self.print_vel()
        # Current movement
        self.move = 0
        self.turn = 0

        # TODO: ROS setup

        # Disable input echoing and line buffering
        self.attr = termios.tcgetattr(sys.stdin)
        self.attr[3] &= ~(termios.ECHO | termios.ICANON)
        # Set non-blocking
        self.attr[6][termios.VMIN] = chr(0) # min num chars
        self.attr[6][termios.VTIME] = chr(10) # timeout (disecs)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.attr)

    def process_arrow_key(self):
        ch = sys.stdin.read(1)
        if (ch != '' and ord(ch) == 91): # potentially an arrow key
            ch = sys.stdin.read(1)
            if (ch != ''):
                key = ord(ch)
                # Update car movement
                if (key == 65): # up
                    self.move = 1
                    self.turn = 0
                elif (key == 66): # down
                    self.move = 0
                    self.turn = 0
                elif (key == 67): # right
                    self.turn = -1
                elif (key == 68): # left
                    self.turn = 1

    def process_key(self, key):
        if (key == 45): # -
            self.lin_vel -= 0.01
            self.print_vel()
        elif (key == 61): # =
            self.lin_vel += 0.01
            self.print_vel()
        elif (key == 91): # [
            self.ang_vel -= 0.01
            self.print_vel()
        elif (key == 93): # ]
            self.ang_vel += 0.01
            self.print_vel()
        elif (key == 27): # esc
            self.process_arrow_key()

        # Calculate signed velocities
        v = self.move * self.lin_vel
        omega = self.turn * self.ang_vel

        # TODO: Publish the velocity command

    def print_vel(self):
        print "[%s] Speed settings:\t%s\t%s" % (self.node_name, self.lin_vel, self.ang_vel)

if __name__ == '__main__':
    # TODO: ROS node init

    # Store original terminal settings
    orig_attr = termios.tcgetattr(sys.stdin)
    try:
        node = KeyboardControl()
        while (True): # TODO: should only loop if the ROS node is still running
            ch = sys.stdin.read(1)
            if (ch != ''):
                node.process_key(ord(ch))
    finally:
        # Restore settings no matter what
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_attr)
