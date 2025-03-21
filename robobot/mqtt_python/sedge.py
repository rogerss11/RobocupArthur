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
# test threshold values (should be okay when calibrated properly)s
# test the values of switching navigatingIntersection with zero speed, just move it by hand
# figure out if adding lead compensator is better
# fine-tune values for PID



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

    # crossing thresholds - average above this is assumed to be crossing line
    crossingThreshold = 800 # grey
    #crossingThreshold = 400 # black ground
    #crossingThreshold = 480 # black ramp

    # line detection values
    position = 0.0 #? what exactly is 0, between 4 and 5th sensor?
    
    lineValid = False
    # start at 10 to avoid going to different mode at the start
    lineValidCnt = 10 # a value up to 20 for most confident line detect, 0 = line isnt valid

    atIntersection = False # if the current reading suggests that we are at an intersection
    atIntersectionCnt = 0  # 20 = at Intersection
    passedIntersections = 0 # how many intersection have we passed
    intersectionPath = ['r', 'r', 'l'] # l = choose left line, r = choose right line
    navigatingIntersection = False # if we are currently navigating an intersection

    average = 0 # avarage edge_n[] value
    high = 0 # highest reflectivity
    low = 0  # the darkest value found in latest sample

    topicLip = ""   
    sendCalibRequest = False # False = Calibrate
    
    # follow line controller
    lineCtrl = False # private

    # my PID values
    Kp = 0.2  # Proportional constant
    Ki = 0.2  # Integral constant
    Kd = 0.12  # Derivative constant
    # PID values from the c++ code (upid)

    # values for ID
    errorSum = 0.0  # Integral term (sum of errors)
    lastError = 0.0  # Previous error for calculating the derivative term
    lastTime = t.time()  # Time of last update to calculate the derivative (dt)

    # Lists to store data for graphing
    error_list = []
    time_list = []
    startingTime = 0


    # lead compensator
    lineTauZ = 1.5
    lineTauP = 0.5

    # Lead pre-calculated factors
    tauP2pT = 1.0
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0

    # old control values 
    lineE1 = 0.0 # old error * Kp (rad/s)
    lineY1 = 0.0 # old control output (rad/s)
    lineY = 0.0  # control output (rad/s)

    # management
    topicRc = ""
    lostLineCnt = 0
    u = 0 # turn rate control signal


    ##########################################################

    def setup(self):
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

    def decode(self, topic, msg):
        # decode MQTT message
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
            self.LineDetect()
            # use to control, if active
            if self.lineCtrl:
              self.followLine()
            # log relevant line sensor data
            flog.write()
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

    # Calculate current position, intersections and so on
    def LineDetect(self):
      sum = 0
      low = int(1000)
      high = int(1)

      # find levels (and average)
      # using normalised readings (0 (no reflection) to 1000 (calibrated white)))
      for i in range(8):
        sum += self.edge_n[i] # for average
        if self.edge_n[i] > high:
          high = self.edge_n[i] # most bright value (floor level)
      self.high = high # most white level

      # print(f"% Edge (sedge.py):: {low}, {high} - what")

      # average white level
      self.average = sum / 8.0

      # is line valid (high above threshold)
      self.lineValid = self.high >= self.lineValidThreshold
      # updates lineValidCnt
      # -1 if theres not a line
      # +1 up to 20 if there is
      if self.lineValid and self.lineValidCnt < 20:
        self.lineValidCnt += 1
      elif not self.lineValid:
        if self.lineValidCnt > 0:
          self.lineValidCnt -= 1  
        else:
          self.lineValidCnt = 0

      # detect if we have a crossing line
      self.atIntersection = self.average >= self.crossingThreshold
      # updates crossingValidCnt
      # -1 if theres not a crossing
      # +1 up to 20 if there is
      if self.atIntersection and self.atIntersectionCnt < 20:
        self.atIntersectionCnt += 1
      elif not self.atIntersection:
        self.atIntersectionCnt -= 1
        if self.atIntersectionCnt < 0:
          self.atIntersectionCnt = 0

      # if we have arrived at an intersection
      if self.atIntersectionCnt == 20: 
        self.navigatingIntersection = True
      # if we have passed the intersection
      elif self.atIntersectionCnt <= 2 and self.navigatingIntersection:
        self.navigatingIntersection = False
        self.passedIntersections += 0 #! change to 1 after testing

      # find position of the line we want to follow
      if self.navigatingIntersection: # if we are navigating an intersection, find the position of the line we want to follow
        self.calculateIntersection()
      else: # otherwise just average the values
        self.calculateLine()


      # print(f"% Edge (sedge.py):: ({self.edge_n[0]} {self.edge_n[1]} {self.edge_n[2]} {self.edge_n[3]} {self.edge_n[4]} {self.edge_n[5]} {self.edge_n[6]}), min={self.low}, high={self.high}, pos={self.position:.2f}.")

    ##########################################################

    #! just a thought, should probably find a better way to do this
    # try and decipher the two lines from the 8 color sensors and follow the one defined in intersectionPath
    def calculateIntersection(self):
      #print("calculateIntersection callled")
      return

      # ignore or values under low
      values = [max(self.edge_n[i] - self.low, 0) for i in range(8)]
      print("values: ", values)

      # iterate from right to left or left to right depending on the intersectionPath
      if self.intersectionPath[self.passedIntersections] == 'l':
        start, end, step = 0, 8, 1
      elif self.intersectionPath[self.passedIntersections] == 'r':
        start, end, step = 8, 0, -1
      else:
        print("Invalid intersectionPath")
        return

      sum = 0
      posSum = 0
      found_nonzero = False  # flag to check if a nonzero value has been encountered
      for i in range(start, end, step):

        if values[i] != 0:
          found_nonzero = True  # Mark that we have seen a nonzero value
        elif found_nonzero and values[i] == 0:
          break  # Stop the loop when we hit 0 after a nonzero value
          
        sum += values[i]
        posSum += (i + 1) * values[i]


      if sum > 0 and self.lineValid:
        """
        using weighted average 
        position= [∑(sensor intensity) * ∑(sensor index)]/ ∑(sensor intensity) - middle(4.5)
        """
        self.position = posSum/sum - 4.5
      else:
        self.position = 0
      
      print("position: ", self.position)  


    
    ##########################################################

    # find line position
    def calculateLine(self):
      sum = 0
      posSum = 0
      for i in range(8):
        # everything more black than 'low' is ignored
        v = self.edge_n[i] - self.low
        if v > 0:
          sum += v
          posSum += (i+1) * v

      if sum > 0 and self.lineValid:
        """
        using weighted average 
        position= [∑(sensor intensity) * ∑(sensor index)]/ ∑(sensor intensity) - middle(4.5)
        """
        self.position = posSum/sum - 4.5
      else:
        self.position = 0

    ##########################################################

    def lineControl(self, velocity, refPosition):
      self.velocity = velocity
      self.refPosition = refPosition # position on the line (0 = middle)
      # velocity 0 is turning off line control
      self.lineCtrl = velocity > 0.001 # is line control active
      pass

    ##########################################################

    def followLine(self):
      from uservice import service
      # some parameters depend on sample time, adjust
      # print(f"LineCtrl:: sample time {self.edge_nInterval}")
      if abs(self.edge_nInterval - self.edgeIntervalSetup) > 2.0: # ms #? why
        self.PIDrecalculate()
        self.edgeIntervalSetup = self.edge_nInterval

      # line to (much) the right gives a line position value.
      # Then the robot is too much to the left.
      # To correct we need a negative turnrate,
      # so sign is OK
      
      # Time difference for derivative and integral calculations
      currentTime = t.time()
      deltaTime = currentTime - self.lastTime

      if deltaTime > 0: # just to make sure
        # Calculate the error between the desired position and the current position
        e = self.refPosition - self.position

        self.errorSum += e * deltaTime  # Sum of errors for integral term
        errorDiff = (e - self.lastError) / deltaTime  # Derivative term

        # PID control output
        self.u = self.Kp * e + self.Ki * self.errorSum + self.Kd * errorDiff

        # Lead filter
        # self.lineY = (self.u * self.tauZ2pT - self.lineE1 * self.tauZ2mT + self.lineY1 * self.tauP2mT)/self.tauP2pT;
        
        self.lineY = self.u
      
        if self.lineY > 4:
          self.lineY = 4
        elif self.lineY < -4:
          self.lineY = -4

        # Save data for graphing
        self.error_list.append(e)
        self.time_list.append(currentTime - self.startingTime)

        # save old values
        self.lineE1 = self.u
        self.lineY1 = self.lineY

        # Save last values
        self.lastError = e
        self.lastTime = currentTime

        # make response
        print("velocity edge: ", self.velocity)
        par = f"{self.velocity:.3f} {self.lineY:.3f} {t.time()}"
        service.send(self.topicRc, par) # send new turn command, maintaining velocity

        # debug print
        if self.edge_nUpdCnt % 20 == 0:
          print(f"% Edge::followLine: ctrl: e={e:.3f}, u={self.u:.3f}, y={self.lineY:.3f} -> {par}")

    ##########################################################

    def PIDrecalculate(self):
      print(f"LineCtrl:: PIDrecalculate: T={self.edgeIntervalSetup:.2f} -> {self.edge_nInterval:.2f} ms")
      Tsec = self.edge_nInterval/1000
      self.tauP2pT = self.lineTauP * 2.0 + Tsec
      self.tauP2mT = self.lineTauP * 2.0 - Tsec
      self.tauZ2pT = self.lineTauZ * 2.0 + Tsec
      self.tauZ2mT = self.lineTauZ * 2.0 - Tsec
      # debug
      print(f"%% Lead: tauZ {self.lineTauZ:.3f} sec, tauP = {self.lineTauP:.3f} sec, T = {self.edge_nInterval:.3f} ms")
      print(f"%%       tauZ2pT = {self.tauZ2pT:.4f}, tauZ2mT = {self.tauZ2mT:.4f}, tauP2pT = {self.tauP2pT:.4f}, tauP2mT = {self.tauP2pT:.4f}")

    ##########################################################

    def terminate(self):
      from uservice import service
      self.need_data = False
      print("% Edge (sedge.py):: turn off line sensor")
      service.send(self.topicLip, "0")
      # try:
      #   self.th.join()
      #   # stop subscription service from Teensy
      #   service.send(service.topicCmd + "T0/sub","livn 0")
      # except:
      #   print("% Edge thread not running")
      print("% Edge (sedge.py):: terminated")
      pass

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


# create the data object
edge = SEdge()

