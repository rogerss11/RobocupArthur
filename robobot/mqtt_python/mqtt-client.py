#!/usr/bin/env python3

# /***************************************************************************
# *   Copyright (C) 2024 by DTU
# *   jcan@dtu.dk
# *
# *
# * The MIT License (MIT)  https://mit-license.org/
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# * and associated documentation files (the “Software”), to deal in the Software without restriction,
# * including without limitation the rights to use, copy, modify, merge, publish, distribute,
# * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
# * is furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in all copies
# * or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# * THE SOFTWARE. */

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
from simu import imu

from drive import *

############################################################


def imageAnalysis(save):
    if cam.useCam:
        ok, img, imgTime = cam.getImage()
        if not ok:  # size(img) == 0):
            if cam.imageFailCnt < 5:
                print("% Failed to get image.")
        else:
            h, w, ch = img.shape
            if not service.args.silent:
                # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
                pass
            edge.paint(img)
            if not gpio.onPi:
                cv.imshow("frame for analysis", img)
            if save:
                fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
                cv.imwrite(fn, img)
                if not service.args.silent:
                    print(f"% Saved image {fn}")
            pass
        pass
    pass


############################################################

stateTime = datetime.now()


def stateTimePassed():
    return (datetime.now() - stateTime).total_seconds()


############################################################


def loop():
    """
    Please talk to Arnau or Roger before touching other things. When in doubt, ask.

    This is the main loop where the route for the robot is implemented. The states are 3 digit numbers
    The first number relates to what part of the route it's from:

    1 --> start (100-199)
            this should end at the intersection already looking at the seesaw (but not on it)
    2 --> seesaw + seesaw golf ball (200 - 299)
            this should end with the seesaw golf ball in the hole
    3 --> top golf ball (300 - 399)
            this should end with the top golf ball in the hole
    4 --> stairs (400 - 499)
            this should end positioned to follow the line towards the extra time
            *** might be skipped to implement 5 instead
    5 --> downramp (500 - 599)
            this should end positioned to follow the line towards the extra time
            *** might be skipped to implement 4 instead
    6 --> axe (600 - 699)
            this should end on the center line looking at birtle gate
    7 --> figure-8 + roundabout (700 - 799)
            this should end on the center line
    8 --> blue ball sorting (800 - 899)
            this should end the run

    9: tests (9000-9999)

    Make each jump from each state inside a mission in increments of 5
    For example

    State 100: move forward
    State 105: turn left
    State 110: raise arm

    This way, if you have to add an extra step in between 100 and 105, you have 4 steps and everything
    is correctly labeled

    """

    # INITIAL PARAMETERS
    from ulog import flog

    gate_dist = 99999
    min_d = 99999
    state = 0
    images = 0
    ledon = True
    tripTime = datetime.now()
    oldstate = -1

    # START
    service.send(service.topicCmd + "T0/leds", "16 30 30 0")  # LED 16: yellow - waiting
    flog.write(state)
    print(f"% Using state {state}")
    # elif not service.args.now:
    #   print("% Ready, press start button")

    # main state machine
    edge.lineControl(0, 0)  # make sure line control is off
    while not (service.stop):
        if state == 0:  # wait for start signal
            start = True  # gpio.start() or service.args.now
            if start:
                print("% Starting")
                service.send(service.topicCmd + "T0/leds", "16 0 0 30")  # blue: running
                service.send(
                    service.topicCmd + "ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)

                state = 100  # ========== START STATE ===============
                # should be 100
                pose.tripBreset()  # use trip counter/timer B

        ############################### START FUNCTIONS (100-199) ###############################

        elif state == 100:  # VOJTA
            pass

        ######################### SEESAW + SEESAW GOLF BALL (200-299) ###########################

        elif state == 200:  # Arnau + Leona
            pass

        ############################ TOP GOLF BALL (300-399) ####################################

        elif state == 300:  # Leona
            pass

        ############################## STAIRS SECTION (400-499) #################################
        #                *** might be skipped to implement 5 instead

        elif state == 400:  # Roger
            pass

        ############################# DOWNRAMP SECTION (500-599) ################################
        #                *** might be skipped to implement 4 instead

        elif state == 500:  # Arnau
            pass

        ############################### AXE SECTION (600-699) ###################################

        elif state == 600:  # Vojta + Arnau + Andrea
            pass

        ####################### FIGURE-8 + ROUNDABOUT SECTION (700-799) #########################

        elif state == 700:  # Roger
            # ------------- PASS BIRTLE (start from line) -------------------------------------------
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            driveXMeters(0.3, vel=0.3)
            driveUntilWall(0.30, ir_id=1, vel=0.0)
            t.sleep(3)
            driveXMeters(1.3, vel=0.3)
            state = 705

        elif state == 705:
            # ------------- CLIMB CIRCLE MISSION -------------------------------------------
            gate_dist = driveUntilWall_measure_gate_dist(0.25, ir_id=1)
            print(f"% gate_dist = {gate_dist:.2f}")
            turnInPlace(63, dir=1)  # turn counter-clockwise 65=90deg
            service.send(service.topicCmd + "T0/servo", "1 -200 200")
            driveXMeters(gate_dist + 0.25)  # drive to extra time
            driveXMeters(-0.30)
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            turnInPlace(25, dir=1)
            climbCircle(40, vel=0.45)
            state = 710  # inside circle

        elif state == 710:
            # ------------- INSIDE CIRCLE MISSION -------------------------------------------
            turnInPlace(38, dir=0, ang_speed=0.4)  # position tangent to the circle
            min_d = driveUntilWall(0.3, ir_id=0, vel=0.1)
            print(f"% min_d = {min_d:.2f}")
            turn_rad = min_d + 0.125  # 0.1 is the 1/2 of the wheel base
            turnInPlace(8, dir=1, ang_speed=0.3)  # Adjust position
            rotateCircle(r=turn_rad, deg=333, dir=1)
            state = 715

        elif state == 715:
            # ------------- LEAVE CIRCLE -------------------------------------------
            driveXMeters(0.3 + min_d, vel=0.3)
            driveUntilWall(0.3, ir_id=1, vel=0.0)
            t.sleep(7)
            driveXMeters(1, vel=0.2)
            driveUntilLine(400)
            turnInPlace(63, dir=1)  # turn counter-clockwise 65=90deg
            state = 720

        ######################### BLUE BALL SORTING SECTION (800-899) ###########################

        elif state == 800:  # Eva + Leona
            pass

        #################################### TESTS (9000-9999) ###################################

        elif state == 9000:
            # ------ TESTS --------------------------------------------------------
            t.sleep(30)
            state = 9100

        else:  # abort
            print(f"% Mission finished/aborted; state={state}")
            break

        # allow openCV to handle imshow (if in use)
        # images are almost useless while turning, but
        # used here to illustrate some image processing (painting)
        if cam.useCam:
            imageAnalysis(False)
            key = cv.waitKey(100)  # ms
            if key > 0:  # e.g. Esc (key=27) pressed with focus on image
                break
        #
        # note state change and reset state timer
        if state != oldstate:
            # flog.write(state)
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            print(f"% State change from {oldstate} to {state}")
            oldstate = state
            stateTime = datetime.now()
        # do not loop too fast
        t.sleep(0.1)
        pass  # end of while loop
    # end of mission, turn LEDs off and stop
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    gpio.set_value(20, 0)
    edge.lineControl(0, 0)  # stop following line
    service.send(service.topicCmd + "ti/rc", "0 0")
    t.sleep(0.05)
    print(f"gate_dist = {gate_dist:.2f}, min_d = {min_d:.2f}")
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
        # service.setup("localhost")  # localhost
        service.setup("10.197.218.235")  # Arthur
        # service.setup("10.197.218.184") # Gandalf
        if service.connected:
            loop()
        service.terminate()
    print("% Main Terminated")
