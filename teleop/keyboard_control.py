#!/usr/bin/env python

import time
import curses

import dagudrive

if __name__ == '__main__':

        dagu = dagudrive.DAGU_Differential_Drive(False, False, False, False, False)

        # start screen to read keys
        screen = curses.initscr()
        curses.cbreak()
        curses.noecho() # no line buffering
        # screen.nodelay(1) # non-blocking
        screen.keypad(1)

        base_angle = 0.0
        increment = 3.14/16

        while True:
            c = screen.getch()
            if (c == 113): # q
                break
            elif (c == 61): # =
                increment *= 2.0
            elif (c == 45): # -
                increment /= 2.0
            elif (c == 91): # [
                base_angle += increment
                base_angle = min(base_angle, 1)
            elif (c == 93): # ]
                base_angle -= increment
                base_angle = max(base_angle, -1)
            elif (c == curses.KEY_LEFT):
                dagu.setSteerAngle(0.2)
            elif (c == curses.KEY_UP):
                dagu.setSteerAngle(base_angle)
                dagu.setSpeed(0.2)
            elif (c == curses.KEY_RIGHT):
                dagu.setSteerAngle(-0.2)
            elif (c == curses.KEY_DOWN):
                dagu.setSpeed(0.0)
                dagu.setSteerAngle(0.0)
            # elif (c > 0):

            time.sleep(0.1)

            screen.clear()
            screen.move(0,0)
            screen.addstr("Base angle: %.2f" % base_angle)
            screen.move(1,0)
            screen.addstr("Increment: %.2f" % increment)

            screen.refresh()

        curses.endwin()
