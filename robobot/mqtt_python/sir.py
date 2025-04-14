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
#bua

import time as t
from datetime import *
from sedge import edge

class SIr:
    ir = [0, 0]
    irUpdCnt = 0
    irTime = datetime.now()
    irInterval = 0
    
    axeActive = False
    axeState = 1
    passedAxe = False
    timeout = 0

    followLineUntilWall = False
    followLineVelocity = 0.0
    WallDistance = 0.0

    ##########################################################
    #################   CONTROL FUNCTIONS   ##################
    ##########################################################

    def startAxe(self):
        self.axeActive = True
        self.passedAxe = False
        self.axeState = 1
        self.timeout = 0
    
    ##########################################################

    def hasPassedAxe(self):
        return self.passedAxe

    ##########################################################

    def setFollowUntilWall(self, velocity, distance):
        self.followLineUntilWall = True
        self.followLineVelocity = velocity
        self.WallDistance = distance
        edge.lineControl(velocity, 0.0)
    
    ##########################################################

    def hasReachedWall(self):
        return self.followLineUntilWall

    ##########################################################

    def terminate(self):
        print("% IR terminated")
        pass
    
    ##########################################################
    #################   CORE FUNCTIONS   #####################
    ##########################################################

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from uservice import service
      loops = 0
      while not service.stop:
        # wait for data to arrive
        t.sleep(0.01)
        if self.irUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IR sensor (sir.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IR sensor (sir.py):: missing data updates after {loops} wait loops (continues).")
          break
        pass
      pass

    ##########################################################
    
    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/ir" or topic == "T0/ird":
          gg = msg.split(" ")
          if (len(gg) >= 3):
            t0 = self.irTime;
            self.irTime = datetime.fromtimestamp(float(gg[0]))
            self.ir[0] = float(gg[1])
            self.ir[1] = float(gg[2])
            t1 = self.irTime;
            if self.irUpdCnt == 2:
              self.irInterval = (t1 -t0).total_seconds()
            else:
              self.irInterval = (self.irInterval * 99 + (t1 - t0).total_seconds()) / 100
            self.irUpdCnt += 1
            # self.print()
        else:
          used = False
        if self.axeActive:
          self.axe()
        elif self.followLineUntilWall:
          self.followLineUntilWallFunc()
        return used
    
    ##########################################################

    def axe(self):
        if self.axeState == 1: # still furter from the object
            if ir.ir[1] < 1:
                edge.lineControl(0.1, 0.0)  # slow down
                self.axeState = 2
        
        elif self.axeState == 2:  # aproches the object -> slow down more
            if ir.ir[1] < 0.5:  # if the object is detected less than 50 cm
                edge.lineControl(0.05, 0.0)  # slow down more
                self.axeState = 3

        elif self.axeState == 3:  # Approaches the object
            if ir.ir[1] < 0.20:  # if the object is detected less than 20 cm
                print("# Object detected less than 20 cm, stops.")
                edge.lineControl(0.0, 0.0)
                self.passedAxe = False
                self.axeState = 4
            elif ir.ir[1] == 1.5:  # if the axe is not in front of us, dont move (we cant see the distance)
                edge.lineControl(0.0, 0.0)
            else: # we can see the axe but we are not close enough
                edge.lineControl(0.05, 0.0)  # continue moving forward

        elif self.axeState == 4:  # Waiting for the object to be removed
            distance = ir.ir[1]
            if distance > 0.3 and self.passedAxe:  # if the object is not detected anymore and the axe has at least once passed
                print("# Object not detected anymore, accelerate to pass the gate.")
                self.timeout = t.time()
                edge.lineControl(0.35, 0.0)
                self.axeState = 5
            else:
                if self.passedAxe == False:
                    print("Axe detected for the first time, waiting for it to pass.")
                self.passedAxe = True

        elif self.axeState == 5:  # Pass the gate axe and stop
            if t.time() - self.timeout > 2: # 2 seconds have passed, should be past gate
                print("# Gate passed, stop.")
                #service.send("robobot/cmd/ti/rc", "0.0 0.0")  # robot stops
                edge.lineControl(0.0, 0.0)
                self.axeState = 99

        else:  # Final state
            print("% Axe completed -------------------------")
            self.axeActive = False

    ##########################################################

    def followLineUntilWallFunc(self):
        if self.ir[1] < self.WallDistance:
            edge.lineControl(0.0, 0.0)
            self.followLineUntilWall = False

    ##########################################################
    #################   DEBUG FUNCTIONS   ####################
    ##########################################################

    def print(self):
      from uservice import service
      print("% IR dist " + str(self.irTime - service.startTime) + " (" +
            str(self.ir[0]) + ", " +
            str(self.ir[1]) + ", " +
            f") {self.irInterval:.4f} sec " +
            str(self.irUpdCnt))

    ##########################################################
    

# create the data object
ir = SIr()