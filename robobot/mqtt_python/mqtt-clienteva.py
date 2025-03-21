#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# Robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from arucode import ArucoDetector  # Import ArucoDetector

#1: stream from stream_server 
#2: scam reads the stream
#3: mqtt gets the image from the stream via scam
#4: mqtt sends the image to the aruco detector
#5: aruco detector detects the markers
#6: mqtt sends the markers to the robot hopefully

# Initialize the ArUco detector
aruco_detector = ArucoDetector()

# Set title of process, so that it is not just called Python
setproctitle("mqtt-client")

############################################################
def imageAnalysis(save):
    """
    Captures and analyzes an image from the Raspberry Pi camera.
    If ArUco markers are detected, they are drawn and sent via MQTT.
    """
    if cam.useCam:
        ok, img, imgTime = cam.getImage()
        if not ok or img is None or img.size == 0:
            print("% Failed to get image. Skipping analysis.")
            return
        
        edge.paint(img)  # Process edge detection

        # Detect ArUco markers
        marker_ids, marker_positions, img = aruco_detector.detect_markers()
        if marker_ids:
            for i, marker_id in enumerate(marker_ids):
                position = marker_positions[i][0].tolist()
                print(f"Detected ArUco Marker ID: {marker_id} at {position}")
                service.send(service.topicCmd + "robot/arucode", f"{marker_id} {position}")

        if not gpio.onPi:
            cv.imshow('Live ArUco Detection', img)  # Show camera feed with detections
            key = cv.waitKey(1)  
            if key == 27:  
                aruco_detector.release()
                sys.exit()

        if save:
            fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
            cv.imwrite(fn, img)


############################################################

stateTime = datetime.now()
def stateTimePassed():
    return (datetime.now() - stateTime).total_seconds()

############################################################

def loop():
    """
    Main loop that controls the robot’s state and runs ArUco detection.
    """
    from ulog import flog
    state = 0
    images = 0
    ledon = True
    tripTime = datetime.now()
    oldstate = -1
    if not service.args.now:
        print("% Ready, press start button")
        service.send(service.topicCmd + "T0/leds", "16 30 30 0")  # LED 16: yellow - waiting

    edge.lineControl(0, 0)  # Ensure line control is off
    while not (service.stop or gpio.stop()):
        if state == 0:  # Wait for start signal
            start = 1  # gpio.start() or service.args.now
            if start:
                print("% Starting")
                service.send(service.topicCmd + "T0/leds", "16 0 0 30")  # Blue LED - running
                service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
                edge.lineControl(0.25, 0.0)  # Follow line at 0.25m/s
                state = 12  # Follow line until no more line
                pose.tripBreset()
        elif state == 12:  # Following line
            if edge.lineValidCnt == 0 or pose.tripBtimePassed() > 20:
                edge.lineControl(0, 0)  # Stop following line
                pose.tripBreset()
                service.send(service.topicCmd + "ti/rc", "0.1 0.5")  # Turn left
                state = 14  # Turning left
        elif state == 14:  # Turning left
            if pose.tripBh > np.pi / 2 or pose.tripBtimePassed() > 25:
                state = 20  # Finished turning
                service.send(service.topicCmd + "ti/rc", "0 0")  # Stop for images
        elif state == 20:  # Image analysis
            imageAnalysis(images == 2)
            images += 1
        else:  # Abort
            print(f"% Mission finished/aborted; state={state}")
            break

        if cam.useCam:
            imageAnalysis(False)

        if state != oldstate:
            flog.write(state)
            print(f"% State change from {oldstate} to {state}")
            oldstate = state
            stateTime = datetime.now()

        t.sleep(0.05)  # Prevent excessive looping
        service.send(service.topicCmd + "ti/alive", str(service.startTime))
    
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # Turn LEDs off
    gpio.set_value(20, 0)
    edge.lineControl(0, 0)
    service.send(service.topicCmd + "ti/rc", "0 0")
    aruco_detector.release()
    t.sleep(0.05)

############################################################

if __name__ == "__main__":
    print("% Starting")
    service.setup('localhost')  # Arthur
    if service.connected:
        loop()
        service.terminate()
    print("% Main Terminated")
