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
from aruco_new import aru #eva needs this to test
from evas_image import *
import image_analysis as ia

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

    Or see Roger's figure-8 + roundabout

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

    # MAIN STATE MACHINE
    edge.lineControl(0, 0)  # make sure line control is off
    while not (service.stop):   
        if state == 0:  # wait for start signal
            start = True  # gpio.start() or service.args.now
            if start:
                print("% Starting")
                service.send(service.topicCmd + "T0/leds", "16 0 0 30")  # blue: running
                service.send(service.topicCmd + "ti/rc", "0.0 0.0")
                # (forward m/s, turn-rate rad/sec)

                state = 140  # ========== START STATE ===============
                # should be 100 for final run
                # CP1 (checkpoint 1)
                # use trip counter/timer B
                service.send(service.topicCmd + "T0/servo", "1 -900 200")
                pose.tripBreset()
                # 9200 turn test
                # 9210 put arm down test
                # 9213 put arm up test
                # 9220 driveUntilNOLine_auto test
                # 9221 driveUntilNOLine_manual test
                # 9230 driveUntilLine test

        ############################### START FUNCTIONS (100-199) ###############################

        elif state == 100:
            service.send(service.topicCmd + "T0/servo", "1 -900 200")  # arm up
            edge.lineControl(0.225, 0.0)  # m/s and position on line
            edge.adjustSpeed = False  # adjust speed to follow line
            edge.setIgnoreIntersections(True)  # ignore intersections
            edge.stopAtNthIntersection(["r"], 2)
            state = 105
            pose.tripBreset()
        
        elif state == 105:
            if pose.tripBtimePassed() > 7:
                edge.lineControl(0.2, 0)
                edge.setIgnoreIntersections(False)
                state = 110

        elif state == 110:
            if pose.tripBtimePassed() > 11.5:
                edge.adjustSpeed = True
                edge.setIgnoreIntersections(True)
                state = 120
                pose.tripBreset()

        elif state == 120:
            if pose.tripB > 1.88:
                edge.adjustSpeed = False
                state = 130
                pose.tripBreset()

        elif state == 130:
            if pose.tripBtimePassed() > 4.4:
                edge.adjustSpeed = True
                state = 140
                pose.tripBreset()

        elif state == 140:
            if True: #pose.tripB > 1.58:
                edge.stopAtNthIntersection([], 1)
                edge.adjustSpeed = False
                edge.setIgnoreIntersections(False)
                edge.lineControl(0.1, 0)
                state = 150
                pose.tripBreset()

        elif state == 150:  # drive until line
            if edge.hasArrivedAtNthIntersection():
                turnInPlace(40, 0)
                edge.setIgnoreIntersections(True)
                edge.lineControl(0.17, 0)
                state = 160
                pose.tripBreset()

        elif state == 160:
            if pose.tripBtimePassed() > 0.9:
                edge.lineControl(0.0, 0)
                driveXMeters(0.2, 0.1)
                pose.tripBreset()
                edge.lineControl(0.1, 0)
                state = 170

        elif state == 170:  # drive until line
            if pose.tripBtimePassed() > 3.8:
                edge.lineControl(0.0, 0)
                pose.tripBreset()
                state = 200 

        ######################### SEESAW + SEESAW GOLF BALL (200-299) ###########################

        elif state == 200:  # Arnau + Leona
            """
            Revise!!!
            """
            ia.drive2ball(2)  # drive to the orange ball
            state = 220

        elif state == 220:
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            if pose.tripBtimePassed() > 2:
                service.send(service.topicCmd + "T0/servo", "1 2309823809 200")
                pose.tripBreset()
                state = 225

        elif state == 225:
            service.send(service.topicCmd + "T0/servo", "1 -150 200")
            if pose.tripBtimePassed() > 2:
                service.send(service.topicCmd + "T0/servo", "1 2309823809 200")
                pose.tripBreset()
                state = 230

        elif state == 230:
            edge.lineControl(0.05, 0)
            if ir.ir[1] < 0.43 or pose.tripBtimePassed() > 20:  # going down
                # print("end of ramp")
                service.send(service.topicCmd + "T0/servo", "1 -200 300")
                state = 231
                pose.tripBreset()

        elif state == 231:
            if pose.tripBtimePassed() > 0.5:
                service.send(service.topicCmd + "T0/servo", "1 -250 300")
                state = 232
                pose.tripBreset()

        elif state == 232:
            if pose.tripBtimePassed() > 0.3:
                service.send(service.topicCmd + "T0/servo", "1 -350 100")
                state = 235
                pose.tripBreset()

        elif state == 235:
            if pose.tripBtimePassed() > 3:
                service.send(service.topicCmd + "T0/servo", "1 -150 25")
                state = 240
                pose.tripBreset()
                print("end of seesaw")

        elif state == 240:
            edge.lineControl(0.1, 0)
            t.sleep(6.5)
            edge.lineControl(0, 0)
            state = 255

        elif state == 255:
            # service.send(service.topicCmd + "T0/servo", "1 -175 0")
            turnInPlace(63, 1)
            driveXMeters(x=1, vel=0.35)
            driveUntilLine(550)
            turnInPlace(45, dir=1)
            state = 260

        elif state == 260:  # Climb the ramp
            edge.lineControl(0.06, 0)
            t.sleep(1.5)
            print(f"ir: {ir.ir[1]}")
            if ir.ir[1] < 0.38:  # going up
                pose.tripBreset()
                state = 261

        elif state == 261:
            # ------------ ARM MOVEMENT BOTTOM ----------------------
            service.send(service.topicCmd + "T0/servo", "1 -190 0")
            edge.lineControl(0.06, 0)
            if pose.tripBtimePassed() > 0.1:
                pose.tripBreset()
                state = 2615

        elif state == 2615:
            service.send(service.topicCmd + "T0/servo", "1 -210 0")
            if pose.tripBtimePassed() > 0.2:
                pose.tripBreset()
                state = 262

        elif state == 262:
            service.send(service.topicCmd + "T0/servo", "1 -235 0")
            if pose.tripBtimePassed() > 0.3:
                pose.tripBreset()
                state = 2625

        elif state == 2625:
            service.send(service.topicCmd + "T0/servo", "1 -265 0")
            if pose.tripBtimePassed() > 0.3:
                pose.tripBreset()
                state = 263

        elif state == 263:
            service.send(service.topicCmd + "T0/servo", "1 -265 0")
            if pose.tripBtimePassed() > 0.2:
                pose.tripBreset()
                state = 2635

        elif state == 2635:
            service.send(service.topicCmd + "T0/servo", "1 -265 0")
            if pose.tripBtimePassed() > 0.5:
                pose.tripBreset()
                state = 264

        elif state == 264:
            service.send(service.topicCmd + "T0/servo", "1 -100 10")
            if pose.tripBtimePassed() > 3:
                pose.tripBreset()
                state = 265

        elif state == 265:
            # ------------- TOP OF THE RAMP -------------------------------------------
            edge.lineControl(0.25, 0)
            if pose.tripBtimePassed() > 4.5:
                # service.send(service.topicCmd + "T0/servo", "1 -100 10")
                pose.tripBreset()
                state = 267

        elif state == 267:
            # -------------- DETECT GATE AND LOWER ARM --------------
            edge.lineControl(0.1, 0)
            if ir.ir[0] < 0.35:
                service.send(service.topicCmd + "ti/rc", "0.05 0.0")
                service.send(service.topicCmd + "T0/servo", "1 -50 0")
                t.sleep(2)
                service.send(service.topicCmd + "T0/servo", "1 -150 50")
                edge.lineControl(0, 0)
                state = 275

        elif state == 275:  # On top of the ramp
            driveUntilNOLine_manual(0.05, 0)
            driveXMeters(0.05, vel=0.1)
            turnInPlace(8, 0)
            wiggle(width=50, ang_speed=0.75, N_wiggles=3)
            state = 280

        elif state == 280:
            # ------------- BALL IN THE HOLE -------------------------------------------
            driveUntilLine(250, vel=-0.15)
            turnInPlace(50, dir=1)
            edge.stopAtNthIntersection([], 1)
            edge.lineControl(0.1, 0)  # follow line
            t.sleep(4)
            edge.lineControl(0.05, 0)
            state = 290
        
        elif state == 290:
            if edge.ArrivedAtNthIntersection:
                driveXMeters(0.15)
                turnInPlace(35, 1)
                state = 300

        ############################ TOP GOLF BALL (300-399) #################################### 
        
        elif state == 300:  # find the orange ball
            xy = []
            image_ia, ok = ia.imageAnalysis(0)

            if ok:
                xy = ia.ball(image_ia, 1)  # detect orange ball

            # Visualize the ball in the picture
            if len(xy) == 2:
                image_ia = cv.circle(
                    image_ia, xy, radius=10, color=(0, 0, 255), thickness=-1
                )

            # Show the image for debugging
            if not gpio.onPi:
                cv.imshow("frame for analysis", image_ia)

            if (xy == []) & ok:  # no ball detected
                # turn right
                service.send(service.topicCmd + "ti/rc", "0.0 0.3")  # turn right
                t.sleep(0.1)
                service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # stop
            else:
                state = 320  # ball detected

        elif state == 320:  # drive to orange ball
            ia.drive2ball(1)
            state = 330

        elif state == 330:  # move to hole
            driveUntilLine(300, vel=-0.15)
            # edge.lineUpWithLine()
            turnInPlace(deg=50, dir=1)
            driveXMeters(x=0.15)
            edge.setIgnoreIntersections(True)
            edge.lineControl(0.15, 0.0)
            t.sleep(1.5)
            edge.lineControl(0.0, 0.0)

            driveXMeters(x=0.22)
            print("Wiggle")
            speed = 0.5
            turnInPlace(20, dir=1, ang_speed=speed)
            wiggle(width=50, ang_speed=0.75, N_wiggles=3)

            state = 340
        
        elif state == 340: # drive to the stairs
            driveUntilLine(450, vel=-0.2)
            driveXMeters(-0.075)
            driveUntilLine(450, vel=-0.10)
            turnInPlace(70,0)
            state = 400
            
        ############################## STAIRS SECTION (400-499) #################################
        #                *** might be skipped to implement 5 instead

        elif state == 400:  # Arnau + Roger
            service.send(service.topicCmd + "T0/servo", "1 -800 200")
            edge.lineControl(0.05, 0)
            t.sleep(5)
            edge.lineControl(0.0, 0.0)
            edge.setIgnoreIntersections(True)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")
            state = 405
              

        elif state == 405:  # Roger + Arnau
            # ------------- GO DOWN STAIRS -------------------------------------------
            steps = 0
            while steps < 5:
                stairStep(60, 0.1)  # go down one step
                if True: # steps == 1:
                    #print(edge.edge_n)
                    #left_avg = sum(edge.edge_n[:4])
                    #right_avg = sum(edge.edge_n[4:])

                    #if right_avg > left_avg:
                    #    print("Line is to the right, turning right")
                    #    turnInPlace(5, dir=1)  # turn right
                    #elif left_avg > right_avg:
                    #    print("Line is to the left, turning left")
                    #    turnInPlace(5, dir=0)  # turn left
                    #else:
                    #    print("Line is centered, no turn")

                    
                    sum_values = sum(edge.edge_n)
                    pos_sum = sum((i + 1) * v for i, v in enumerate(edge.edge_n))
                    position = (pos_sum / sum_values - 4.4) # middle is 4.5 but the sensor seems to be more on the left side of the robot

                    direction = 1 if position > 0 else 0

                    print("position: ", position, " -> turn ", abs(position), " to ", "left" if direction == 0 else "right") 
                    
                    turnInPlace(abs(position)*1.4, direction)  # turn left or right based on how much of center we are

                steps += 1
                print(f"% ----------------- steps = {steps} ------------")

            state = 410

        elif state == 410:
            edge.lineValidCnt = 0
            edge.lineControl(0.05, 0)
            t.sleep(0.1)
            edge.lineControl(0, 0)
            if edge.lineValidCnt <= 3:
                print("not on line -> look for line on the left side first")
                turnInPlace(50, dir=0)  # turn to the left
                edge.setDriveUntilLine(0.15, 300)
                pose.tripBreset()
                state = 411

            else:
                print("ended up on line")
                state = 415
                
        elif state == 411:
            if edge.reachedLine():
                print("found line on the left side")
                driveXMeters(0.025, 0.1)
                turnInPlace(45, dir=1)  # turn to the right
                state = 415

            elif pose.tripBtimePassed() > 1: # turn all the way around
                print("didnt dind line on the left side, turn around and look for it on the right side")
                turnInPlace(110, dir=1)  # turn to the right
                edge.setDriveUntilLine(0.15, 300)
                state = 412
        
        elif state == 412:
            if edge.reachedLine():
                print("found line on the right side")
                driveXMeters(0.025, 0.1)
                turnInPlace(45, dir=0)  # turn to the left
                state = 415

        elif state == 415:
            edge.lineControl(0.1)  # follow line
            t.sleep(6)
            edge.lineControl(0.05)  # follow line
            t.sleep(3)
            edge.lineControl(0, 0)  # stop following line
            pose.tripBreset()
            driveXMeters(0.5)
            pose.tripBreset()
            state = 420

        elif state == 420:
            driveUntilLine(500,0.2)
            state = 600



        # once down the stairs, code can be adapted from the downramp section

        ############################# DOWNRAMP SECTION (500-599) ################################
        #                *** might be skipped to implement 4 instead

        elif state == 500:  # Arnau
            pass

        # missing step from hole to being on the curved line at the top

        elif state == 520:  # START OF DOWNRAMP.
            # assuming it's already on the line at the top of the ramp
            # code works best when starting in the middle of the curve
            edge.lineControl(0.2, 0)
            if pose.tripBtimePassed() > 11:
                edge.lineControl(0, 0)
                pose.tripBreset()
                state = 525

        elif (
            state == 525
        ):  # go straight to skip the trident intersection next to the end
            service.send(service.topicCmd + "ti/rc", "0.2    0.0")
            if pose.tripBtimePassed() > 6:
                service.send(service.topicCmd + "ti/rc", "0 0")
                pose.tripBreset()
                state = 530

        elif (
            state == 530
        ):  # turn to have the line somewhere in front after the intersection
            service.send(service.topicCmd + "ti/rc", "0 0.5")
            if pose.tripBtimePassed() > 3:
                service.send(service.topicCmd + "ti/rc", "0 0")
                pose.tripBreset()
                state = 535

        elif state == 535:  # driveUntilLine
            driveUntilLine()
            pose.tripBreset()
            state = 540

        elif state == 545:
            service.send(service.topicCmd + "ti/rc", "0 -0.5")
            if pose.tripBtimePassed() > 2:
                service.send(service.topicCmd + "ti/rc", "0 0")
                pose.tripBreset()
                state = 550

        elif state == 550:  # slooooowly follow line so that it has time to align itself
            edge.lineControl(0.05, 0)
            if pose.tripBtimePassed() > 2:
                state = 600  # go to axe section

        ############################### AXE SECTION (600-699) ###################################

        # Vojta + Arnau + Andrea
        elif state == 600:  # follow line until end
            service.send(service.topicCmd + "ti/servo", "1 -900 0")
            edge.setIgnoreIntersections(True)
            edge.lineControl(0.1, 0)
            t.sleep(2)
            edge.lineControl(0.2, 0)
            pose.tripBreset()

        elif state == 601:
            if edge.lineValidCnt == 0:
                state = 605
            elif pose.tripBtimePassed() > 2:
                edge.lineControl(0.1, 0)
                state = 605


        elif state == 605:
            if edge.lineValidCnt == 0:  # when line finishes, stop
                edge.lineControl(0, 0)
                state = 610
                pose.tripBreset()

        elif state == 610:
            service.send(
                service.topicCmd + "ti/rc", "0 0.2"
            )  # turn slightly to not miss the axe line
            if pose.tripBtimePassed() > 1:
                service.send(service.topicCmd + "ti/rc", "0 0")
                state = 615
                pose.tripBreset()

        elif state == 615:
            edge.setDriveUntilLine(0.25, 300)
            state = 620

        elif state == 620:  # drive until line
            if edge.reachedLine():
                turnInPlace(60, 0)  # turn to the axe line
                state = 630
                pose.tripBreset()

        elif state == 630:  # set it to drive through the axe
            ir.axeActive = True
            edge.lineControl(0.1, 0.0)
            state = 635

        elif state == 635:  # wait for it to get past the axe
            if not ir.axeActive:
                edge.setIgnoreIntersections(False)
                edge.stopAtNthIntersection([], 1)
                edge.lineControl(0.05, 0.0)
                state = 640

        elif state == 640:
            if edge.hasArrivedAtNthIntersection():
                turnInPlace(63, 0)
                driveXMeters(0.05, 0.1)
                edge.stopAtNthIntersection([], 1)
                edge.lineControl(0.1, 0)
                t.sleep(2)
                edge.lineControl(0.5, 0)
                state = 650

        elif state == 650:
            if edge.hasArrivedAtNthIntersection():
                turnInPlace(10, 1)
                driveXMeters(0.05, 0.1)
                turnInPlace(60, 1)
                t.sleep(0.5)
                state = 660

        elif state == 660:
            ok = False
            while not ok:
                img, ok = ia.imageAnalysis(0)

            xy = ia.ball(img,2)

            if xy == []:
                pass
                #turnInPlace(0, 1)

            else:
                xy[0] = xy[0] - 10 # we need to be facing the e rather than middle of the poster
                Status = ia.move_middle(xy, 2, 0)

                if Status == 0:
                    state = 700

        

        ####################### FIGURE-8 + ROUNDABOUT SECTION (700-799) #########################

        elif state == 700:  # Roger
            # ------------- PASS BIRTLE (start from line) -------------------------------------------
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            driveXMeters(0.45, vel=0.3)
            driveUntilWall(0.30, ir_id=1, vel=0.0)
            t.sleep(3)
            driveXMeters(1.45, vel=0.3)
            state = 705

        elif state == 705:
            # ------------- CLIMB CIRCLE MISSION -------------------------------------------
            gate_dist = driveUntilWall_measure_gate_dist(0.30, ir_id=1)
            print(f"% gate_dist = {gate_dist:.2f}")
            turnInPlace(63, dir=1)  # turn counter-clockwise 65=90deg
            driveXMeters(gate_dist)
            turnInPlace(25, dir=1)
            climbCircle(45, vel=0.45)
            state = 710  # inside circle

        elif state == 710:
            # ------------- INSIDE CIRCLE MISSION -------------------------------------------
            turnInPlace(43, dir=0, ang_speed=0.4)  # position tangent to the circle
            min_d = driveUntilWall(0.3, ir_id=0, vel=0.1)
            print(f"% min_d = {min_d:.2f}")
            turn_rad = min_d + 0.13  # 0.1 is the 1/2 of the wheel base
            turnInPlace(8, dir=1, ang_speed=0.3)  # Adjust position
            rotateCircle(r=turn_rad, deg=310, dir=1)
            state = 715

        elif state == 715:
            # ------------- LEAVE CIRCLE -------------------------------------------

            driveXMeters(min_d + 0.30, vel=0.3)

            state = 720  # go to blue ball sorting section

        elif state == 720:
            driveUntilWall(0.3, ir_id=1, vel=0.0) # waiting for birtle
            t.sleep(0.7) # wait for birtle to pass
            driveUntilLine(300)
            turnInPlace(40, dir=1)  # turn clockwise 65=90deg
            edge.lineControl(0.2, 0)  # follow line
            t.sleep(1.5)
            edge.lineControl(0, 0)
            driveUntilWall(0.3, ir_id=1, vel=0.0) # waiting for birtle
            t.sleep(1) # wait for birtle to pass

            edge.lineControl(0.1, 0)  # follow line
            edge.stopAtNthIntersection(['l'], 1)
            t.sleep(5)
            edge.lineControl(0, 0)  # stop following line
            
            turnInPlace(63,0)
            driveXMeters(0.2,0.2)
            driveUntilLine(300)

            state = 800

        ######################### BLUE BALL SORTING SECTION (800-899) ###########################

        elif state == 800:  # turn left at intersection, raise arm and hit basket 
             service.send(service.topicCmd + "T0/servo", "1 -600 0")
             edge.lineControl(0.07, 0)
             edge.stopAtNthIntersection([], 1)
             t.sleep(0.05)
             if edge.hasArrivedAtNthIntersection():
                 print("Move after intersection")
                 edge.lineControl(0, 0)  # stop following line
                 turnInPlace(45, 0, 0.4)  # turn left
                 #edge.lineControl(0.1, 0)  # follow line
                 driveXMeters(0.2, 0.2)
                 turnInPlace(20, 0, 0.5)  # turn left
                 turnInPlace(20, 1, 0.5)  # turn left
                 service.send(service.topicCmd + "T0/servo", "1 -400000 0")
                 state = 805
        
        elif state == 805: #turn backwards and do an arc
             aru.after_hitting_basket()
             #turnInPlace(20, 1, 0.2)
             driveXMeters(0.1, 0.2)
             state = 810

        elif state == 810:  # find the blue ball
            xy = []
            image_ia, ok = ia.imageAnalysis(0)

            if ok:
                xy = ia.ball(image_ia, 0)  # detect blue ball

            if xy == []:  # no ball detected
                turnInPlace(20, 1, 0.2) 
            else:
                state = 811

        elif state == 811:  # drive to the blue oval ball
            status = ia.drive2ball(0)
            state = 815
 
        elif state == 815:
            img = imageAnalysis(0)
            #service.send(service.topicCmd + "T0/servo", "1 -4500000 0")
            aru.after_catching_blue_ball(img)
            #aru.turn_left_until_ID13_disappears(img)
            state = 99

        elif state == 820:
            aru.start_looking_for_ID16(img)
            state = 99 

        elif state == 850: #Finishing
            ia.servo_up()
            print("Go back")
            driveXMeters(-0.1)
            print("Find line")
            driveUntilLine(300,-0.2) #drive back until the line
            turnInPlace(63, 1)
            print("Line Control")
            edge.setIgnoreIntersections(True)
            edge.adjustSpeed = True
            edge.lineControl(0.3, 0)
            t.sleep(6)
            edge.lineControl(0, 0)
            state = 900
            # How can we detect, that we finished? (Distance or Aruco?)

        #################################### TESTS (9000-9999) ###################################

        elif state == 9000:
            # ------ TESTS --------------------------------------------------------
            t.sleep(30)
            state = 9100

        elif state == 9200:  # TURNING TEST
            service.send(service.topicCmd + "ti/rc", "0.0 1")
            if pose.tripBtimePassed() > 2:
                service.send(service.topicCmd + "ti/rc", "0.0 0")
                state = 9238529837

        elif state == 9210:  # put arm down
            service.send(service.topicCmd + "T0/servo", "1 -150 200")
            if pose.tripBtimePassed() > 3:
                state = 9211

        elif state == 9211:  # put arm down
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            if pose.tripBtimePassed() > 3:
                state = 9212

        elif state == 9212:  # put arm down
            service.send(service.topicCmd + "T0/servo", "1 -150 200")
            if pose.tripBtimePassed() > 3:
                state = 921124534

        elif state == 9213:  # put arm up
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            if pose.tripBtimePassed() > 3:
                state = 92122346264

        elif state == 9220:  # driveUntilNOLine test
            driveUntilNOLine_auto(0.5, 300)
            state = 2455234

        elif state == 9230:  # driveUntilLine test
            driveUntilLine(300)
            state = 9231

        elif state == 9231:
            pose.tripBreset()
            driveXMeters(-0.05)
            service.send(service.topicCmd + "T0/servo", "1 -900 200")
            if pose.tripBtimePassed() > 3:
                state = 9232

        elif state == 9232:
            driveXMeters(-0.2)
            state = 423154

        elif state == 9800:  # blue ball catch test
            pass

        elif state == 9900:  # take one picture
            ia.servo_up()
            t.sleep(0.5)
            ia.imageAnalysis(1)
            images += 1
            t.sleep(2)
            # blink LED
            if ledon:
                service.send(service.topicCmd + "T0/leds", "16 0 64 0")
                gpio.set_value(20, 1)
            else:
                service.send(service.topicCmd + "T0/leds", "16 0 30 30")
                gpio.set_value(20, 0)
            ledon = not ledon
            # finished?
            if images >= 1 or (not cam.useCam) or stateTimePassed() > 100:
                images = 0
                state = 99
            pass

        else:  # abort
            print(f"% Mission finished/aborted; state={state}")
            break

        # allow openCV to handle imshow (if in use)
        # images are almost useless while turning, but
        # used here to illustrate some image processing (painting)
        if cam.useCam:
            ia.imageAnalysis(False)
            key = cv.waitKey(100)  # ms
            if key > 0:  # e.g. Esc (key=27) pressed with focus on image
                break

        # note state change and reset state timer
        if state != oldstate:
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
