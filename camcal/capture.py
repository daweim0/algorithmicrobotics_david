#!/usr/bin/env python

# Source <https://picamera.readthedocs.io/en/release-1.13/recipes1.html>

from time import sleep
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (2592, 1944) #(1024, 768)
camera.start_preview()
# Camera warm-up time
sleep(2)

# Paramters
base = 'left'
counter = 1

while True:
    line = raw_input('')
    if (line == ''):
        filename = ("%s%02d.jpg" % (base, counter))
        camera.capture(filename)
        counter += 1
        print "Captured " + filename
    else:
        break
