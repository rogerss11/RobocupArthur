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

import time as t
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
from ulog import flog
from drive import *

############################################################

def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

############################################################

stateTime = datetime.now() # define globally in mqqt-client.py
def stateTimePassed(): # how much time has passed since last state change
  return (datetime.now() - stateTime).total_seconds()

############################################################

def loop():
  state = 0 # current state
  images = 0 # number of images taken
  ledon = True # LED on/off
  tripTime = datetime.now()
  oldstate = -1 # previous state
  startTime = t.time() # time since beginning of mission

  service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
  if service.args.meter:
    state = 101 # run 1m
  elif service.args.pi:
    state = 102 # run 1m

  #elif service.args.usestate > 0:
  #  state = service.args.usestate

  print(f"% Using state {state}")
  # elif not service.args.now:
  #   print("% Ready, press start button")

  edge.lineControl(0, 0) # make sure line control is off

  # main state machine
  while not (service.stop): # main loop (until red stop button is pressed, or stop signal received)

    #if t.time() - startTime >= 200:  # time limit for mission
    #  print("% Mission finished due to time limit")
    #  break

    if state == 0: # starting state (waiting for start signal or --now)
      if not service.args.distance or ir.ir[0] < 0.1: # if -d was used when starting mqtt-client, wait for someone to 'touch' side IR sensor
        print("% Starting")
        service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        # follow line (at 0.25cm/s)
        edge.lineControl(0.2, 0.0) # m/s and position on line
        state = 100
        pose.tripBreset()

    elif state == 12: # following line
      if edge.lineValidCnt == 0 or pose.tripBtimePassed() > 10: # no line or following for too long
        edge.lineControl(0.25,0) # stop following line
        pose.tripBreset()
        service.send(service.topicCmd + "ti/rc","0.1 0.5") # turn left
        state = 14 # turn left
        
    elif state == 14: # turning left
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 10:
        state = 20 # finished
        service.send(service.topicCmd + "ti/rc","0 0") # stop for images
      print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")

    elif state == 20: # image analysis
      imageAnalysis(images == 2)
      images += 1
      # blink LED
      if ledon:
        service.send(service.topicCmd + "T0/leds","16 0 64 0")
        gpio.set_value(20, 1)
      else:
        service.send(service.topicCmd + "T0/leds","16 0 30 30")
        gpio.set_value(20, 0)
      ledon = not ledon
      # finished?
      if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
        images = 0
        state = 99
      pass

    ###### MY TESTING STATES #######
    # line testing
    elif state == 100:  
            edge.lineControl(0.185, 0.0) # m/s and position on line
            edge.adjustSpeed = False # adjust speed to follow line
            edge.stopAtNthIntersection(["r"], 2)
            state = 110
            pose.tripBreset()
            state = 110
        
    elif state == 110:
        if pose.tripBtimePassed() > 14:
            edge.adjustSpeed = True
            state = 120

    elif state == 120:  # drive until line
        if edge.hasArrivedAtNthIntersection():
            t.sleep(1)
            #driveXMeters(0.025, 0.1)
            turnInPlace(40, 0)
            t.sleep(1)
            driveXMeters(0.1, 0.1)
            t.sleep(1)
            edge.lineControl(0.05, 0)
            edge.setIgnoreIntersection(1)
            pose.tripBreset()
            state = 130
    
    elif state == 130:  # drive until line
        if pose.tripBtimePassed()  > 3:
            edge.lineControl(0.0, 0)
            pose.tripBreset()
            state = 140

#    elif state == 110:
#      if pose.tripBtimePassed() > 2000:
#        edge.lineControl(0.2, 0.0)
#        state = 111
#
#    elif state == 111:
#      if pose.tripBtimePassed() > 18:
#        edge.lineControl(0.25, 0.0)
#        state = 112
#    
#    elif state == 112:
#      if pose.tripBtimePassed() > 22.5:
#        edge.lineControl(0.38, 0.0)
#        state = 113
#      
#    elif state == 113:
#      if pose.tripBtimePassed() > 28:
#        edge.lineControl(0.05, 0.0)
#        state = 114
#    
#    elif state == 114:
#      if pose.tripBtimePassed() > 45:
#        edge.lineControl(0.0, 0.0)
#        state = 115


    # color sensor printing
    elif state == 120:
      #edge.print()
      edge.printn()
      print(edge.edge_n)
      #edge.printnw()
      t.sleep(0.5)

    # at intersection testing values
    elif state == 130:
      edge.printn()
      print("% AtIntersectionCnt: ", edge.atIntersectionCnt, ", navigatingIntersection: ", edge.navigatingIntersection)
      t.sleep(0.5)    

    elif state == 139: # start lining up with white line
      edge.shouldLineUp = True
      state = 140
    elif state == 140: # wait for it to line up with white line
      if not edge.shouldLineUp:
        print("Done lining up")
        state = 99
    
    elif state == 150: # pass axe
        print("start axe")
        edge.lineControl(0.2, 0.0) # stop following line
        ir.axeActive = True
        state = 151
    elif state == 151: # wait for axe to pass
        if not ir.axeActive:
          state = 99
    
    elif state == 160:
      edge.stopAtIntersection = True
      if edge.lineCtrl == False:
        state = 99
        
    elif state == 161:
      t.sleep(3)
      driveXMeters(0.1)
      t.sleep(3)
      turnInPlace(90, -1)
      t.sleep(3)
      driveXMeters(-0.1)
      t.sleep(3)
      state = 162
    
    elif state == 162: # start lining up with white line
      edge.setLineUpWithLine()
      state = 163

    elif state == 163: # wait for it to line up with white line
      if not edge.shouldLineUp:
        state = 99

    elif state == 170:
      driveUntilLineAndTurn(0.1, 'left')
      print("dooooooone")
      edge.stopAtNthIntersection(['l'], 2)
      edge.lineControl(0.1, 0.0) # m/s and position on line
      state = 171
    elif state == 171:
      if edge.lineCtrl == False:
        state = 172
    elif state == 172:
      driveXMeters(0.1)
      t.sleep(1)
      turnInPlace(60, 0)
      t.sleep(1)
      driveXMeters(0.2)
      t.sleep(1)
      driveUntilLine()
      state = 99
    
    elif state == 180:
        ir.setFollowUntilWall(0.1, 0.32)
        state = 181
    elif state == 181:
        if not ir.followLineUntilWall:
          state = 99

    else: # abort
      print(f"% Mission finished/aborted; state={state}")
      break

    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    #if (cam.useCam):
    #  imageAnalysis(False)
    #  key = cv.waitKey(100) # ms
    #  if key > 0: # e.g. Esc (key=27) pressed with focus on image
    #    break

    # note state change and reset state timer
    if state != oldstate:
      # flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()

    # do not loop too fast
    t.sleep(0.1)
    
    pass # end of while loop

  # end of mission, turn LEDs off and stop
  edge.plot_error() # plot error (temporary for PID tuning)
  service.send(service.topicCmd + "T0/leds","16 0 0 0") 
  gpio.set_value(20, 0)
  edge.lineControl(0,0) # stop following line
  service.send(service.topicCmd + "ti/rc","0 0")
  t.sleep(0.05)
  pass

############################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      # set title of process, so that it is not just called Python
      setproctitle("mqtt-client")
      print("% Starting")
      # where is the MQTT data server:
      # service.setup('localhost') # localhost
      service.setup('10.197.218.235')
      #service.setup('10.197.218.11') #gladnalf
      if service.connected:
        loop()
      service.terminate()
    print("% Main Terminated")
