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

#import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from eva_drive import turnInPlace, driveUntilLine, driveXMeters

#from arucode import ArucoDetector  # Import ArucoDetector
from aruco_test import ArucoDetector  # Import ArucoDetector
aruco_detector = ArucoDetector()


# set title of process, so that it is not just called Python
setproctitle("mqtt-client")

############################################################


def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()

    if not ok:  # size(img) == 0
      if cam.imageFailCnt < 5:
        print("Failed to get image.")
      return None  

    h, w, ch = img.shape
    if not service.args.silent:
      pass 
    edge.paint(img)
    if not gpio.onPi:
      cv.imshow('frame for analysis', img)
      cv.waitKey(1)
    return img

############################################################

stateTime = datetime.now()
def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()

############################################################

def loop():
  from ulog import flog
  state = 0
  images = 0
  ledon = True
  tripTime = datetime.now()
  oldstate = -1
  if not service.args.now:
    print("% Ready, press start button")
    service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
  # main state machine
  edge.lineControl(0, 0) # make sure line control is off
  while not service.stop:
    if state == 0: # wait for start signal
      start = 1 #gpio.start() or service.args.now
      if start:
        print("% Starting")
        service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turnrate rad/sec)
        # follow line (at 0.25cm/s)
        #edge.lineControl(0.25, 0.0) # m/s and position on line -2.0..2.0
        state = 120 # until no more line
        pose.tripBreset() # use trip counter/timer B
    elif state == 12: # following line
      if edge.lineValidCnt == 0 or pose.tripBtimePassed() > 20:
        # no more line
        edge.lineControl(0,0) # stop following line
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","0.1 0.5") # turn left
        state = 14 # turn left
    elif state == 14: # turning left
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 25:
        state = 420 # finished   =17 go look for line
        service.send(service.topicCmd + "ti/rc","0 0") # stop for images
      print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")

    elif state == 420:
      service.send(service.topicCmd + "T0/servo", "1 -150 0")
      if pose.tripBtimePassed() > 3:
        state = 20
    
    ########################
    elif state == 120: 
      img = imageAnalysis(0)  # get the image
      aruco_detector.turn_left(img)
      #driveUntilLine()
      #service.send(service.topicCmd + "T0/servo", "1 -150 0")
      driveXMeters(0.3, 0.2)
      aruco_detector.find_and_orient_to_B(img)
      if pose.tripBtimePassed()>2:
        state = 99 
    elif state == 130:
      #edge.followLine()
      #t.sleep(0.2)  # short delay for stabilization
      #edge.pathStart()
      service.send(service.topicCmd + "T0/servo", "1 -450 0")
      edge.lineControl(0.09, 0) # speed and position
      edge.stopAtNthIntersection('l', 1)
      pose.tripBreset()
      edge.lineControl(0.08,0)
      #followline
      #intersection, first one, turn left
      #lift servo a bit
      #drive forward and crash 
      #drive backwards
      #drive to the right 
      #look for balls 
      if pose.tripBtimePassed()>2:
        state = 99 
    elif state == 20: # image analysis
      #service.send(service.topicCmd + "T0/servo", "1 -150 0")
      img = imageAnalysis(0)  # get the image
      if img is not None:
          aruco_detector.find_and_orient_to_B(img)
          aruco_detector.drive_to_line_and_find_C() #turn left until it sees aruo C, 15 
          #change to drive to line - V will fix so its straight. 
          #edge.stopAtNthIntersection(['l'], 2) #left, number 2 intersection 
          #edge.lineControl(0.08, 0) #speed and position
          state = 99 
    ###################################
      if not cam.useCam or stateTimePassed() > 120:
          state = 99
      # blink LED
      if ledon:
        service.send(service.topicCmd + "T0/leds","16 0 64 0")
        gpio.set_value(20, 1)
      else:
        service.send(service.topicCmd + "T0/leds","16 0 30 30")
        gpio.set_value(20, 0)
      ledon = not ledon
      # finished?
    else: # abort
      print(f"%Mission finished/aborted; state={state}")
      break
    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    if (cam.useCam):
      imageAnalysis(False)
      key = cv.waitKey(100) # ms
      if key > 0: # e.g. Esc (key=27) pressed with focus on image
        break
    #
    # note state change and reset state timer
    if state != oldstate:
      flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.05)
    # tell interface that we are alive
    service.send(service.topicCmd + "ti/alive",str(service.startTime))
    pass # end of while loop
  # end of mission, turn LEDs off and stop
  service.send(service.topicCmd + "T0/leds","16 0 0 0") 
  gpio.set_value(20, 0)
  edge.lineControl(0,0) # stop following line
  service.send(service.topicCmd + "ti/rc","0 0")
  t.sleep(0.05)
  pass

############################################################

if __name__ == "__main__":
    print("% Starting")
    # where is the MQTT data server:
    #service.setup('localhost') # localhost
    service.setup('10.197.218.235') #Arthur
    if service.connected:
      loop()
      service.terminate()
    print("% Main Terminated")

""""
    elif state == 20: # image analysis
      img = imageAnalysis(0) #getting the image
      #print("first part fail")
      if img is not None: 
        #print("second part fail")
        #detect the aruco markers
        marker_ids, marker_positions, img = aruco_detector.detect_markers(img)
        if marker_ids:
          #print("third part fail")
          for i, marker_id in enumerate(marker_ids):
            position = marker_positions[i][0].tolist()
            print(f"Detected ArUco Marker ID: {marker_id} at {position}")
            service.send(service.topicCmd + "robot/arucode", f"{marker_id} {position}")
            # Show the image with detected markers
          if not gpio.onPi:
            cv.imshow('Live ArUco Detection', img)
      if not cam.useCam or stateTimePassed() > 20:
         state = 99"


      elif state == 20: # image analysis
      img = imageAnalysis(0)  # get the image
      if img is not None:
          ids, corners, centers, angles, img = aruco_detector.detect_markers(img)
          #print("Hello there")
          if ids:
              for i, marker_id in enumerate(ids):
                  center = centers[i]
                  angle = angles[i]
                  print(f"ID: {marker_id} | Center: {center} | Angle: {angle:.2f} deg")
                  # --- Stop if close enough ---
                  #state = 99
                  #OPS!!!!! 
                  aruco_detector.find_B(img, ids, centers)
                  state = 99 
          if not gpio.onPi:
              cv.imshow('Live ArUco Detection', img)
              cv.waitKey(1)
    """