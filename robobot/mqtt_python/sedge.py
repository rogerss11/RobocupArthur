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


######## INFROMATION #########
# start with -w, --white -> Calibrate white tape level (python3 mqtt-client.py -w)


#TODO
#! recalibrate -w and test line following and intersections with lowered color sensor
#! speed based on error or rather speed up for the long line after start
#! navigate final intersection
# improve PID (Kp, Ki, Kd)
# Optional: Lead Compensator (if needed)

from datetime import *
import time as t
from threading import Thread
import cv2 as cv
from ulog import flog
import matplotlib.pyplot as plt # graph for Ziegler-Nichols method

class SEdge:
    # raw AD values 
    edge = [0, 0, 0 , 0, 0, 0, 0, 0]
    edgeUpdCnt = 0
    edgeTime = datetime.now()
    edgeInterval = 0

    # normalizing white values
    edge_n_w = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_n_wUpdCnt = 0
    edge_n_wTime = datetime.now()

    # normalized after white calibration
    edge_n = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_nUpdCnt = 0
    edge_nTime = datetime.now()
    edge_nInterval = 0
    edgeIntervalSetup = 0.1

    # line detection levels
    lineValidThreshold = 750 # 1000 is calibrated white
    # level for relevant white values
    low = lineValidThreshold - 100
    woodenLow = 800 #! test this

    # line detection values
    position = 0.0
    
    lineValid = False
    # start at 10 to avoid going to different mode at the start
    lineValidCnt = 10 # a value up to 20 for most confident line detect, 0 = line isnt valid

    ignoreIntersections = False # if we should ignore intersections
    stopAtIntersectionN = 0 # stop when we reach nth intersection

    atIntersectionCntMaxValue = 3 #! test this value
    atIntersection = False # if the current reading suggests that we are at an intersection
    atIntersectionCnt = 0  # 20 = at Intersection
    passedIntersections = 0 # how many intersection have we passed
    intersectionPath = ['r', 'r', 'r', 'r'] # l = choose left line, r = choose right line, m = choose middle line
    navigatingIntersection = False # if we are currently navigating an intersection
    ArrivedAtNthIntersection = False

    adjustSpeed = False # if we should adjust speed based on error

    average = 0 # avarage edge_n[] value
    high = 0 # highest reflectivity

    topicLip = ""   
    sendCalibRequest = False # False = Calibrate
    
    # follow line controller
    lineCtrl = False # private

    # my PID values
    Kp = 0.35           # 0.32        # keep as is
    Ki = 0.15           # 0.15        # slightly reduce integral accumulation near zero
    Kd = 0.2            # 0.17        # increase derivative for better anticipation

    errorCutoff = 0.06  # Error cutoff for integral and derivitive term
    integral_limit = 1.5
    alpha_d = 0.1 # smoothing factor for derivative term

    lineTauZ = 0.6   # increase lead zero to anticipate error crossing earlier
    lineTauP = 0.5   # decrease pole slightly for sharper lead effect


    #lineTauZ = 0.1  # unchanged
    #lineTauP = 0.3  # increase to soften filter

    # values for ID
    errorSum = 0.0  # Integral term (sum of errors)
    lastError = 0.0  # Previous error for calculating the derivative term
    lastErrorDiff = 0.0  # Previous derivative term for filtering
    smoothedErrorDiff = 0.0
    lastTime = t.time()  # Time of last update to calculate the derivative (dt)

    # Lists to store data for graphing
    error_list = []
    time_list = []
    startingTime = 0

    # Lead pre-calculated factors
    tauP2pT = 1.0
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0

    # old control values 
    lineE1 = 0.0 # old error * Kp (rad/s)
    lineY1 = 0.0 # old control output (rad/s)
    lineY = 0.0  # control output (rad/s)

    shouldLineUp = False # if we should line up with white line in front of us
    last_cmd = None # last command sent to the robot (used for lineUpWithLine)

    # management
    topicRc = ""
    lostLineCnt = 0
    u = 0 # turn rate control signal


    ##########################################################
    #################   CONTROL FUNCTIONS   ##################
    ##########################################################

    def resetIntersection(self):
        """ Reset the intersection detection variables. """
        self.atIntersection = False
        self.atIntersectionCnt = 0
        self.navigatingIntersection = False

    ##########################################################

    def setIgnoreIntersections(self, ignore):
        """ Set whether to ignore intersections or not. """
        self.ignoreIntersections = ignore
        if ignore:
            self.resetIntersection()

    ##########################################################

    def setIntersectionPath(self, path):
        """ Set the path for next intersections. """
        self.intersectionPath = path
        self.resetIntersection()
        self.passedIntersections = 0
    
    ##########################################################

    def stopAtNthIntersection(self, path, n):
        """ Stop at the nth intersection following the current path. """
        if (len(path) != n-1):
            print("Path length does not match the number of intersections.")
            return
        self.setIntersectionPath(path)
        self.stopAtIntersectionN = n
        self.ArrivedAtNthIntersection = False

    ##########################################################

    def hasArrivedAtNthIntersection(self):
       return self.ArrivedAtNthIntersection

    ##########################################################

    def setLineUpWithLine(self):
        """ Line up with the white line in front of us. """
        self.shouldLineUp = True
        self.lineControl(0.0, 0.0) # turn off line control

    ##########################################################

    def lineControl(self, velocity, refPosition=0):
      """ Set the line control parameters. """
      from uservice import service
      self.velocity = velocity
      self.refPosition = refPosition # position on the line (0 = middle)
      # velocity 0 is turning off line control
      self.lineCtrl = velocity > 0.001 # is line control active
      if not self.lineCtrl:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # stop robot

    ##########################################################
    
    def terminate(self):
      """ Terminate the line sensor. """
      from uservice import service
      self.need_data = False
      #print("% Edge (sedge.py):: turn off line sensor")
      service.send(self.topicLip, "0")
      #print("% Edge (sedge.py):: terminated")

    ##########################################################
    #################   CORE FUNCTIONS   #####################
    ##########################################################

    def setup(self):
      """ Setup the color sensors. """
      from uservice import service
      sendBlack = False # False = Calibrate
      loops = 0
      self.startingTime = t.time()
      
      # turn line sensor on (command 'lip 1')
      print("% Edge (sedge.py):: turns on line sensor")
      self.topicLip = service.topicCmd + "T0/lip"
      service.send(self.topicLip, "1")
      # topic for (remote) control
      self.topicRc = service.topicCmd + "ti/rc"
      # request fast update (every 3 ms)
      service.send(service.topicCmd + "T0/sub","livn 10")
      # request data
      while not service.stop:
        t.sleep(0.02)

        # white calibrate requested
        if service.args.white:
          
          # sendBlack = 0 -> set lowest value ("black") to 0
          if not sendBlack:
            print("% Edge (sedge.py):: set lowest value (black) to 0")
            topic = service.topicCmd + "T0/litb"  
            param = "0 0 0 0 0 0 0 0"
            sendBlack = service.send(topic, param)

          # require at least 3 updates (reflectivity data is stable)
          elif self.edgeUpdCnt < 3:
            # request raw AD reflectivity
            service.send(service.topicCmd + "T0/livi"," ")
            pass

          # calibration is requested
          elif not self.sendCalibRequest:
            print("% Edge (sedge.py):: sending calibration request")
            # send calibration request, averaged over 100 samples
            service.send(service.topicCmd + "T0/liwi","")
            t.sleep(0.02)
            service.send(service.topicCmd + "T0/licw","100")
            # allow communication to settle
            # wait for calibration to finish (each sample takes 1-2 ms)
            t.sleep(0.25)
            # save the calibration as new default
            service.send(service.topicCmd + "T0/eew","")
            self.sendCalibRequest = True
            # ask for new white values
            service.send(service.topicCmd + "T0/liwi","")
            t.sleep(0.02)

          # no calibration requested
          else:
            t.sleep(0.25)
            service.args.white = False
            print(f"% Edge (sedge.py):: calibration should be fine, got {self.edge_n_wUpdCnt} updates - terminates")
            # terminate mission
            service.terminate()

        elif self.edge_n_wUpdCnt == 0:
          # get calibrated white value
          service.send(service.topicCmd + "T0/liwi"," ")
          pass

        elif self.edge_nUpdCnt == 0:
          # wait for line sensor data
          pass

        else:
          print(f"% Edge (sedge.py):: got data stream; after {loops} loops")
          break
        loops += 1
        if loops > 30:
          print(f"% Edge (sedge.py):: got no data after {loops} (continues edge_n_wUpdCnt={self.edge_n_wUpdCnt}, edgeUpdCnt={self.edgeUpdCnt}, edge_nUpdCnt={self.edge_nUpdCnt})")
          break
      pass

    ##########################################################

    def decode(self, topic, msg):
      """ Decode the MQTT message. """
      used = True
      if topic == "T0/liv": # raw AD value
        from uservice import service
        gg = msg.split(" ")
        if (len(gg) >= 4):
          t0 = self.edgeTime
          self.edgeTime = datetime.fromtimestamp(float(gg[0]))
          self.edge[0] = int(gg[1])
          self.edge[1] = int(gg[2])
          self.edge[2] = int(gg[3])
          self.edge[3] = int(gg[4])
          self.edge[4] = int(gg[5])
          self.edge[5] = int(gg[6])
          self.edge[6] = int(gg[7])
          self.edge[7] = int(gg[8])
          t1 = self.edgeTime
          if self.edgeUpdCnt == 2:
            self.edgeInterval = (t1 -t0).total_seconds()*1000
          elif self.edgeUpdCnt > 2:
            self.edgeInterval = (self.edgeInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
          self.edgeUpdCnt += 1
          # self.print()
      elif topic == "T0/livn": # normalized after calibration range (0..1000)
        from uservice import service
        gg = msg.split(" ")
        if (len(gg) >= 4):
          t0 = self.edge_nTime
          self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
          self.edge_n[0] = int(gg[1])
          self.edge_n[1] = int(gg[2])
          self.edge_n[2] = int(gg[3])
          self.edge_n[3] = int(gg[4])
          self.edge_n[4] = int(gg[5])
          self.edge_n[5] = int(gg[6])
          self.edge_n[6] = int(gg[7])
          self.edge_n[7] = int(gg[8])
          t1 = self.edge_nTime
          if self.edge_nUpdCnt == 2:
            self.edge_nInterval = (t1 -t0).total_seconds()*1000
          elif self.edge_nUpdCnt > 2:
            self.edge_nInterval = (self.edge_nInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
          self.edge_nUpdCnt += 1
          # calculate line position - actually center of gravity of white value
          # - missing edge detection
          # got new normalized values
          # debug save as a remark with timestamp
          # flog.writeDataString(f" {msg}");
          # use to control, if active
          if self.lineCtrl:
              self.LineDetect()
          if self.lineCtrl:
              self.followLine()
          # if we want to line up with white line
          if self.shouldLineUp:
              self.lineUpWithLine()
          flog.write()
          # log relevant line sensor data
          #self.printn()
      elif topic == "T0/liw": # get white level
        from uservice import service
        gg = msg.split(" ")
        if (len(gg) >= 4):
          self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
          self.edge_n_w[0] = int(gg[1])
          self.edge_n_w[1] = int(gg[2])
          self.edge_n_w[2] = int(gg[3])
          self.edge_n_w[3] = int(gg[4])
          self.edge_n_w[4] = int(gg[5])
          self.edge_n_w[5] = int(gg[6])
          self.edge_n_w[6] = int(gg[7])
          self.edge_n_w[7] = int(gg[8])
          self.edge_n_wUpdCnt += 1
          # self.printnw()
      else:
        used = False
      return used

    ##########################################################

    def LineDetect(self):
        """ Detect the line and calculate the position. """
        high = 0
        values = [0] * 8
        valuesAboveZero = 0

        # Find lines above threshold, find highest value
        for i in range(8):
            currentValue = self.edge_n[i]
            high = max(high, currentValue)  # Find highest value

            thresholdedValue = max(currentValue - self.low, 0)  # Remove low values
            values[i] = thresholdedValue  # Update value in list
            valuesAboveZero += thresholdedValue > 0  # Count values above 0

        # Check if the line is valid (high above threshold)
        self.lineValid = high >= self.lineValidThreshold

        # Update lineValidCnt
        # -1 if there's no line, +1 up to 20 if there is
        self.lineValidCnt = (
            min(self.lineValidCnt + 1, 20)
            if self.lineValid
            else max(self.lineValidCnt - 1, 0)
        )

        if not self.ignoreIntersections:
        # Detect if we have a crossing line
            if valuesAboveZero >= 4: #! maybe change to 4
                self.atIntersection = True
            else:
                self.atIntersection = False
            # Updates crossingValidCnt: -1 if there's no crossing, +1 up to atIntersectionCntMaxValue if there is
            self.atIntersectionCnt = (
                min(self.atIntersectionCnt + 1, self.atIntersectionCntMaxValue)
                if self.atIntersection
                else max(self.atIntersectionCnt - 1, 0)
            )

            if self.atIntersectionCnt == self.atIntersectionCntMaxValue:
                if not self.navigatingIntersection:
                    print("started navigating intersection", self.passedIntersections)
                self.navigatingIntersection = True

            # If we have passed the intersection
            elif self.atIntersectionCnt == 0 and self.navigatingIntersection:
                print("finised navigating intersection", self.passedIntersections)
                self.navigatingIntersection = False
                self.passedIntersections += 1
          
        # print(self.edge_n, " -> ", values)
        # print(valuesAboveZero, self.atIntersectionCnt, self.navigatingIntersection)

        # If we are currently at an intersection
        if self.navigatingIntersection and not self.ignoreIntersections:
            if self.stopAtIntersectionN == self.passedIntersections + 1: # if we want to stop at next intersection
                self.lineControl(0, 0)
                self.position = 0
                self.ArrivedAtNthIntersection = True
                return

            path = self.intersectionPath[self.passedIntersections] 

            # If we arrived at a T intersection
            if valuesAboveZero == 8:
                print("arrived at T")
                self.position = {'l': -4, 'r': 4}.get(path, 0)
                if path == 'm':
                    print("Straight at T intersection - Invalid intersectionPath")
                return
            
            #! maybe adjust the 4 to 5?
            # if we arrived at a straight and left intersection
            elif all(value > 0 for value in values[:4]): 
                print("arrived at left and straight intersection")
                if path == 'l':
                    self.position = -4
                elif path == 'r':
                    self.position = 0
                elif path == 'm':
                    print("invalid intersectionPath")
                    self.position = 0
                return
              
            # if we arrived at a straight and right intersection
            elif all(value > 0 for value in values[-4:]):
                print("arrived at right and straight intersection")
                if path == 'l':
                    self.position = 0
                elif path == 'r':
                    self.position = 4
                elif path == 'm':
                    print("invalid intersectionPath")
                    self.position = 0
                return
               
            # normal split intersection
            else:
                # If we are navigating a normal intersection (split)
                ignoreFirst = path == 'm'  # Ignore first line (if we want to go middle)
                start, end, step = (0, 8, 1) if path != 'r' else (7, -1, -1)
                # Calculate values for position calculation
                sum_values, pos_sum = 0, 0
                nonZeroCount = 0

                for i in range(start, end, step):
                    if values[i] > 0:
                        sum_values += values[i]
                        pos_sum += (i + 1) * values[i]
                        nonZeroCount += 1
                    elif nonZeroCount and values[i] == 0:
                        if ignoreFirst:  # Ignore the first line and go for the second one (middle when there's 3)
                            ignoreFirst = False
                            sum_values, pos_sum = 0, 0
                        else:
                            break  # Stop the loop when we hit 0 after a nonzero value
                
                # Using weighted average for position calculation
                # position = [∑(sensor intensity) * ∑(sensor index)] / ∑(sensor intensity) - middle(4.5)
                self.position = (pos_sum / sum_values - 4.5) if sum_values > 0 and self.lineValid else 0
                #print(f"values: {values} position: {self.position:.2f}, path: {path}")

        # Normal line calculation
        else:
            sum_values = sum(self.edge_n)
            pos_sum = sum((i + 1) * v for i, v in enumerate(self.edge_n))
            self.position = (pos_sum / sum_values - 4.5) if sum_values > 0 and self.lineValid else 0
            
    ##########################################################

    def followLine(self):
        """ Follow the line using PID-Lead control. """
        from uservice import service

        # some parameters depend on sample time, adjust
        if abs(self.edge_nInterval - self.edgeIntervalSetup) > 1.0: # ms #? why
          self.PIDrecalculate()
          self.edgeIntervalSetup = self.edge_nInterval

        # line to (much) the right gives a line position value.
        # Then the robot is too much to the left.
        # To correct we need a negative turnrate,
        # so sign is OK
        
        # Time difference for derivative and integral calculations
        deltaTime = max(self.edge_nInterval / 1000, 0.001)  # in seconds

        # Calculate the error between the desired position and the current position
        #print(self.edge_n, " -> ", self.position)
        e = self.refPosition - self.position

        p_term = self.Kp * e
        p_term = (p_term * self.tauZ2pT - self.lineE1 * self.tauZ2mT + self.lineY1 * self.tauP2mT)/self.tauP2pT

        if abs(e) < self.errorCutoff:
            i_term = 0
            self.errorSum = 0
            d_term = 0
        
        else:
            self.errorSum += e * deltaTime
            self.errorSum = max(min(self.errorSum, self.integral_limit), -self.integral_limit)
            i_term = self.Ki * self.errorSum

            errorDiff_raw = (e - self.lastError) / deltaTime
            self.smoothedErrorDiff = (self.alpha_d * getattr(self, 'smoothedErrorDiff', 0) + (1 - self.alpha_d) * errorDiff_raw)
            d_term = self.Kd * self.smoothedErrorDiff
            d_term = max(min(d_term, 2.0), -2.0)  # You can tune this clamp

        # Final control signal
        self.u = p_term + i_term + d_term

        if abs(e) < self.errorCutoff and abs(self.u) < 0.05:
            self.u = 0.05 if e < 0 else -0.05  # Tiny nudge in the correct direction

        self.lineY = self.u

        #print(f"time: {self.edge_nTime.timestamp() - self.startingTime} error: {e:.4f} p_term: {p_term:.4f} d_term {d_term:.4f} i_term: {i_term:.4f} u: {self.u:.4f}")

        self.lineY = max(min(self.lineY, 4), -4)  # Limit the control signal to [-4, 4] rad/s

        # save old values
        self.lineE1 = p_term
        self.lineY1 = self.lineY
        self.lastError = e

        # Save data for graphing
        self.error_list.append(e)
        self.time_list.append(self.edge_nTime.timestamp() - self.startingTime)

        if self.adjustSpeed:
            # Adjust speed based on error
            # Linear scaling based on error where:
            # error = 0 -> 200% of velocity
            # error >= 1 -> 30% of velocity
            scale_factor = 2.0 - (1.7 * e)

            scale_factor = max(0.3, min(2.0, scale_factor))  # limit the range

            adjusted_speed = self.velocity * scale_factor
        else:
            adjusted_speed = self.velocity
        
        # make response
        par = f"{adjusted_speed:.3f} {self.lineY:.3f} {t.time()}"
        service.send(self.topicRc, par) # send new turn command, maintaining velocity

    ##########################################################

    def PIDrecalculate(self):
      """ Recalculate the PID parameters. """
      print(f"LineCtrl:: PIDrecalculate: T={self.edgeIntervalSetup:.2f} -> {self.edge_nInterval:.2f} ms")
      Tsec = self.edge_nInterval/1000
      self.tauP2pT = self.lineTauP * 2.0 + Tsec
      self.tauP2mT = self.lineTauP * 2.0 - Tsec
      self.tauZ2pT = self.lineTauZ * 2.0 + Tsec
      self.tauZ2mT = self.lineTauZ * 2.0 - Tsec
      # debug
      # print(f"%% Lead: tauZ {self.lineTauZ:.3f} sec, tauP = {self.lineTauP:.3f} sec, T = {self.edge_nInterval:.3f} ms")
      # print(f"%%       tauZ2pT = {self.tauZ2pT:.4f}, tauZ2mT = {self.tauZ2mT:.4f}, tauP2pT = {self.tauP2pT:.4f}, tauP2mT = {self.tauP2pT:.4f}")

    ##########################################################

    def lineUpWithLine(self):
        print(self.edge_n)
        """ Line up with the white line in front of us. """
        # positive turnrate -> turn left
        from uservice import service
        speed = 0.4
        left = self.edge_n[0] > self.lineValidThreshold #! maybe replace with low
        right = self.edge_n[7] > self.lineValidThreshold

        if left and right: # we have lined up
            service.send("robobot/cmd/ti/rc","0 0")
            self.shouldLineUp = False
            return
        elif left:
            cmd = f"0 {speed:.3f}"  # turn left
        elif right:
            cmd = f"0 {-speed:.3f}"  # turn right
        else:
            cmd = "0.05 0"  # go straight

        if cmd != self.last_cmd:
            #print("cmd changed")
            service.send("robobot/cmd/ti/rc", cmd)
            self.last_cmd = cmd

    ##########################################################
    #################   DEBUG FUNCTIONS   ####################
    ##########################################################

    def print(self): # print raw values
      from uservice import service
      print("% Edge (sedge.py):: " + str(self.edgeTime - service.startTime) +
            f" ({self.edge[0]}, " +
            f"{self.edge[1]}, " +
            f"{self.edge[2]}, " +
            f"{self.edge[3]}, " +
            f"{self.edge[4]}, " +
            f"{self.edge[5]}, " +
            f"{self.edge[6]}, " +
            f"{self.edge[7]})" +
            f" {self.edgeInterval:.2f} ms " +
            str(self.edgeUpdCnt))
      
    def printn(self): # print normalized values
      from uservice import service
      print("% Edge (sedge.py):: normalized " + str(self.edge_nTime - service.startTime) +
            f" ({self.edge_n[0]}, " +
            f"{self.edge_n[1]}, " +
            f"{self.edge_n[2]}, " +
            f"{self.edge_n[3]}, " +
            f"{self.edge_n[4]}, " +
            f"{self.edge_n[5]}, " +
            f"{self.edge_n[6]}, " +
            f"{self.edge_n[7]})" +
            f" {self.edge_nInterval:.2f} ms " +
            f" {self.position:.2f} " +
            str(self.edge_nUpdCnt))
      
    def printnw(self): # print normalized white values
      from uservice import service
      print("% Edge (sedge.py):: white level " + str(self.edge_n_wTime) +
            f" ({self.edge_n_w[0]}, " +
            f"{self.edge_n_w[1]}, " +
            f"{self.edge_n_w[2]}, " +
            f"{self.edge_n_w[3]}, " +
            f"{self.edge_n_w[4]}, " +
            f"{self.edge_n_w[5]}, " +
            f"{self.edge_n_w[6]}, " +
            f"{self.edge_n_w[7]}) " +
            str(self.edge_n_wUpdCnt))

    ##########################################################

    def paint(self, img): # paint sensor values etc. onto an image
      h, w, ch = img.shape
      pl = int(h - h/4) # base position bottom (most positive y)
      st = int(w/10) # distance between sensors
      gh = int(h/2) # graph height
      x = st # base position left
      y = pl
      dtuGreen = (0x35, 0x88, 0) # BGR
      dtuBlue = (0xea, 0x3e, 0x2f)
      dtuRed = (0x00, 0x00, 0x99)
      dtuPurple = (0x8e, 0x23, 0x77)
      # paint baseline
      cv.line(img, (x,y), (int(x + 7*st), int(y)), dtuGreen, thickness=1, lineType=8)
      # paint calibrated white line (top)
      cv.line(img, (x,int(y-gh)), (int(x + 7*st), int(y-gh)), dtuGreen, thickness=1, lineType=8)
      # paint threshold line for line valid
      cv.line(img, (x,int(y-gh*self.lineValidThreshold/1000.0)), (int(x + 7*st), int(y-gh*self.lineValidThreshold/1000.0)), dtuBlue, thickness=1, lineType=4)
      # draw current sensor readings
      for i in range(8):
        y = int(pl - self.edge_n[i]/1000 * gh)
        cv.drawMarker(img, (x,y), dtuRed, markerType=cv.MARKER_STAR, thickness=2, line_type=8, markerSize = 10)
        x += st
      # paint line position
      pixP = int((self.position + 4)*st)
      cv.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuRed, thickness=3, lineType=4)
      # paint low line position
      pixL = pl - int(gh * self.low/1000)
      cv.line(img, (st, pixL), (st*8, pixL), dtuRed, thickness=1, lineType=4)
      # some axis marking
      cv.putText(img, "Left", (st,pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "Right", (int(st+6*st),pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "White (1000)", (int(st),pl - gh - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      if self.atIntersection:
        cv.putText(img, "Crossing", (int(st),int(pl - 20)), cv.FONT_HERSHEY_PLAIN, 1, dtuRed, thickness=2)
    
    ##########################################################
    
    def plot_error(self, filename="pid_error_plot.png"):
      # Plot error over time
      plt.figure(figsize=(10, 5))
      plt.plot(self.time_list, self.error_list, label="Error")
      plt.axhline(y=0, color='black', linestyle='--')  # Reference line at zero
      plt.xlabel("Time (s)")
      plt.ylabel("Error")
      plt.title("PID Error Over Time")
      plt.legend()
      plt.grid()
      
      # Save plot to file
      plt.savefig(filename, dpi=300, bbox_inches="tight")
      plt.close()  # Close the figure to free memory
    
   ##########################################################


# create the data object
edge = SEdge()
