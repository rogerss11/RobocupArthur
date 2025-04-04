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

# import sys
# import threading
import time as t

# import select
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


def driveOneMeter():
    state = 0
    pose.tripBreset()
    print("% Driving 1m -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            service.send(
                "robobot/cmd/ti/rc", "0.2 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(
                f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving 1m ------------------------- end")


def driveToLine():
    state = 0
    pose.tripBreset()
    dist_to_line = 0
    print("% Driving to line ---------------------- right ir start ---")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # forward towards line
            if ir.ir[0] < 0.2:
                service.send(
                    "robobot/cmd/ti/rc", "0.2 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                service.send("robobot/cmd/T0/lognow", "3")  # (start Teensy log)
                state = 1
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            if edge.lineValidCnt > 4:
                # start follow line
                edge.lineControl(0.2, 0)
                dist_to_line = pose.tripB
                pose.tripBreset()
                print(" to state 10")
                state = 10
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                print(" to state 99")
                state = 99
        elif state == 10:
            if edge.lineValidCnt < 2:
                edge.lineControl(0, 0)
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                print(" to state 2")
                state = 2
        else:
            print(
                f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
        t.sleep(0.01)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving to line ------------------------- end")


def driveTurnPi():
    state = 0
    pose.tripBreset()
    print("% Driving a Pi turn -------------------------")
    service.send(service.topicCmd + "T0/leds", "16 0 100 0")  # green
    while not (service.stop):
        if state == 0:  # wait for start signal
            service.send(
                "robobot/cmd/ti/rc", "0.2 0.5"
            )  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
                service.send(
                    "robobot/cmd/ti/rc", "0.0 0.0"
                )  # (forward m/s, turn-rate rad/sec)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                state = 99
        else:
            print(
                f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds"
            )
            service.send(
                "robobot/cmd/ti/rc", "0.0 0.0"
            )  # (forward m/s, turn-rate rad/sec)
            break
        print(
            f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds"
        )
        t.sleep(0.05)
    pass
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # end
    print("% Driving a Pi turn ------------------------- end")


def loop():
    from ulog import flog

    state = 0
    images = 0
    ledon = True
    tripTime = datetime.now()
    oldstate = -1
    service.send(service.topicCmd + "T0/leds", "16 30 30 0")  # LED 16: yellow - waiting
    flog.write(state)
    # if service.args.meter:
    #     state = 101  # run 1m
    # elif service.args.pi:
    #     state = 102  # run 1m
    # elif service.args.usestate > 0:
    #    state = service.args.usestate
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

                # follow line (at 0.25cm/s)
                #edge.lineControl(0.25, 0.0)  # m/s and position on line -2.0..2.0
                state = 71  
                pose.tripBreset()  # use trip counter/timer B
        elif state == 12:  # following line
            if edge.lineValidCnt == 0 or pose.tripBtimePassed() > 10:
                # no more line
                edge.lineControl(0, 0)  # stop following line
                pose.tripBreset()
                service.send(service.topicCmd + "ti/rc", "0.1 0.5")  # turn left
                state = 14  # turn left
        elif state == 14:  # turning left
            if pose.tripBh > np.pi / 2 or pose.tripBtimePassed() > 10:
                state = 20  # finished   =17 go look for line
                service.send(service.topicCmd + "ti/rc", "0 0")  # stop for images
            print(
                f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}"
            )
        elif state == 20:  # image analysis
            imageAnalysis(images == 2)
            images += 1
            # blink LED
            if ledon:
                service.send(service.topicCmd + "T0/leds", "16 0 64 0")
                gpio.set_value(20, 1)
            else:
                service.send(service.topicCmd + "T0/leds", "16 0 30 30")
                gpio.set_value(20, 0)
            ledon = not ledon
            # finished?
            if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
                images = 0
                state = 99
            pass
        elif state == 101:
            driveOneMeter()
            state = 100
        elif state == 102:
            driveTurnPi()
            state = 100
        elif state == 103:
            driveToLine()
            state = 100
        elif state == 70:  # Mission 360
            # driveXMeters(0.5)
            # orientateToWall(ir_id=1, dir=0, tolerance=0.05, window=20)
            driveUntilWall(0.3, ir_id=1)
            # followWall(0.5, d_front=0.3)
            turnInPlace(90, dir=1)  # turn counter-clockwise 65=90
            climbCircle(80, vel=0.3)
            state = 71
        elif state == 71:  # Mission 360
            #driveUntilLine()
            #turnInPlace(90, dir=1)  # turn counter-clockwise 65=90
            Axe()
            state = 99
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
        # Print gyroscope values every 10 updates
        if imu.gyroUpdCnt % 10 == 0:
            imu.setup()
            imu.print()
        print(f"% IMU gyro {imu.gyro[0]} {imu.gyro[1]} {imu.gyro[2]}")
        t.sleep(0.1)
        pass  # end of while loop
    # end of mission, turn LEDs off and stop
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    gpio.set_value(20, 0)
    edge.lineControl(0, 0)  # stop following line
    service.send(service.topicCmd + "ti/rc", "0 0")
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
        # service.setup("localhost")  # localhost
        service.setup("10.197.218.235")  # Arthur
        #service.setup("10.197.218.184")
        # service.setup('10.197.217.81') # Juniper
        # service.setup('10.197.217.80') # Newton
        # service.setup("bode.local")  # Bode
        if service.connected:
            loop()
        service.terminate()
    print("% Main Terminated")




#IR due valori, primo per il sensore laterale, secondo per il frontale, il valore è in metri 
# quindi quando è 0.16 indica 16 cm, it works. quando non percepisce niente spara 1.5