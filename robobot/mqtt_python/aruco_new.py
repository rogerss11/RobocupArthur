import cv2 as cv
import numpy as np
from eva_drive import turnInPlace, driveXMeters, driveUntilLine, driveUntilWall
from scam import cam
from uservice import service
from sedge import edge 
from uservice import service
from spose import pose
import time as t
from datetime import datetime
from sir import ir

class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters()

        # --- Load calibration --- #this does not really work tho
        calib_data = np.load("calibration_data.npz")
        self.mtx = calib_data['mtx']
        self.dist = calib_data['dist']

    # --- Detect markers --- # it detects the arucos and gives the center and angle of the marker
    def detect_markers(self, frame): 
        if frame is None or frame.size == 0:
            print("Warning: Invalid image received.")
            return None, None, None, None, frame

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        centers = []
        angles = []

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i, marker_id in enumerate(ids.flatten()):

                # --- Compute center ---
                center_x = int(np.mean(corners[i][0][:, 0]))
                center_y = int(np.mean(corners[i][0][:, 1]))
                centers.append((center_x, center_y))

                # --- Compute angle ---
                dx = corners[i][0][1][0] - corners[i][0][0][0]
                dy = corners[i][0][1][1] - corners[i][0][0][1]
                angle = np.arctan2(dy, dx) * 180 / np.pi
                angles.append(angle)

                # --- Draw center and angle ---
                cv.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)  # green dot
                print(f"ID: {marker_id} | Center: ({center_x}, {center_y}) | Angle: {angle:.2f} deg")

            return ids.flatten().tolist(), corners, centers, angles, frame
        else:
            return None, None, None, None, frame
        
    # --- This will turn the robot to center the marker in the camera --- # NOT TESTED
    def turning_to_center(self):
        #center coordinate, center angles
        image_center = (400, 300)
        center_tolerance = 30 #pixels
        angle_tolerance = 5 #degrees
        while True: 
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                continue
            ids, corners, centers, angles, img = self.detect_markers(img)
            if not ids: 
                print("No marker detected. Turning a bit.")
                service.send(service.topicCmd + "ti/rc", "0 -0.2")  # search left
                continue
            dx = centers[0] - image_center[0] #positive, too far right
            dy = centers[1] - image_center[1] #positive, too far down
            print(f"Marker center offset: dx={dx}, dy={dy}, angle={angles:.2f}")

            # Adjust heading based on position
            if abs(dx) > center_tolerance:
                if dx > 0:
                    print("Marker is to the right → turn right")
                    service.send(service.topicCmd + "ti/rc", "0 0.2")  # rotate right
                else:
                    print("Marker is to the left → turn left")
                    service.send(service.topicCmd + "ti/rc", "0 -0.2")  # rotate left

            # Adjust heading based on angle
            elif abs(angles) > angle_tolerance:
                if angles > 0:
                    print("Marker angled to the right → turn right slightly")
                    service.send(service.topicCmd + "ti/rc", "0 0.1")
                else:
                    print("Marker angled to the left → turn left slightly")
                    service.send(service.topicCmd + "ti/rc", "0 -0.1")

            else:
                print("Marker centered and straight → stop")
                service.send(service.topicCmd + "ti/rc", "0 0")
                break
            t.sleep(0.1)  # Small delay to avoid rapid commands
        
    # --- Find and orient to marker B (ID 13) --- #
    def after_hitting_basket(self):
        driveXMeters(x=-0.2, vel=0.2) #back away from the basket
        service.send(service.topicCmd + "T0/servo", "1 -150 0") #lower servo, remove when we merge with Leona blue ball detection, the arm needs to be up 
        turnInPlace(deg=45, dir=0, ang_speed=0.5)  #turn a little the left before doing arc 

        print("% Doing arc turn")

        # Turn with forward speed + angular velocity
        forward_speed = 0.2   # m/s
        turn_rate = -0.3      # rad/s (negative = clockwise)

        pose.tripBreset() #reset time
        state = 0
        while not service.stop:
            if state == 0:
                service.send("robobot/cmd/ti/rc", f"{forward_speed} {turn_rate}")
                state = 1
            elif state == 1:
                if pose.tripBtimePassed() > 2.5:  # adjust timing based on how big arc you want
                    service.send("robobot/cmd/ti/rc", "0.0 0.0")
                    state = 2
            elif state == 2:
                print(f"# Finished arc in {pose.tripBtimePassed():.2f} sec")
                break
            t.sleep(0.05)
          
    # --- This will look for the B marker, drive to right distance in front of it --- # but this might make it go out of image?
    def after_catching_blue_ball(self, img): #TEST TRIGGER 
        found_13 = False
        service.send(service.topicCmd + "T0/servo", "1 -450 0")
        print("🔍 Looking for ID 13...")

        while not found_13:
            turnInPlace(deg=30, dir=1, ang_speed=0.5) #turn clockwise 
        
            # Get image and check
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break

            ids, _, _, _, img = self.detect_markers(img) #detect aruco markers
            if ids:
                print(f"Detected: {ids}")
                if 13 in ids:
                    print("✅ Found ID 13!")
                    service.send(service.topicCmd + "ti/rc", "0 0")  # stop turning

                    #TRIGGER SENSOR
                    min_d = ir.ir[1] # Get distance from IR sensor
                    print("Distance captured: ", min_d)
                    if min_d > 0.7: # if the object is detected less than 70 cm
                        driveUntilWall(0.7, 0.2, 0.2)  # drive until wall
                        service.send(service.topicCmd + "T0/servo", "1 -450 0")
                        break  
                    else: 
                        driveUntilWall(0.7, 0.2, -0.2) #back away until distance@
                        service.send(service.topicCmd + "T0/servo", "1 -450 0")
                        break  
                    #found_13 = True
                    #break

            t.sleep(0.3)  # Pause briefly before switching direction

    # --- After finding ID 13, turn left until it disappears --- #
    def turn_left_until_ID13_disappears(self, img):
        while True:
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break

            ids, _, _, _, img = self.detect_markers(img)
            if ids:
                print(f"Still detecting: {ids}")
                print("🔄 Turning left until ID 13 disappears...")
                service.send(service.topicCmd + "ti/rc", "0 0.25")  # Slow turn left
                if 13 not in ids:
                    service.send(service.topicCmd + "ti/rc", "0 0")
                    print("❌ ID 13 gone — stop turning") 
                    break
            else:
                print("No marker visible — this should not happen")
                break

            t.sleep(0.1)

        # Final step
        print("Turning a little to the left to look for line")
        turnInPlace(15,0)
        print("🚗 Driving until line")
        driveUntilLine()
        edge.lineControl(0.03, 0) # speed and position (slooow line control so it can catch up)

    # --- This will drive on the line and look for the ID 16 marker --- #
    def start_looking_for_ID16(self, img):
        print("Looking for ID 16...")
        edge.lineControl(0.09, 0)
        #if pose.tripBtimePassed()>1: #if more than 3 secs pass, stop line control, look right and check aruco
        t.sleep(9) #change based on when we want to turn to look for ID 16. Do we even need this? can we hardcode to turn and drive towards line after x secs? 
        edge.lineControl(0,0) #stop line control
        turnInPlace(59, dir=1) #turn towards the arucos

        # Refresh image
        ok, img, _ = cam.getImage()
        if not ok:
            print("Could not get image.")
            return
        ids, _, _, _, img = self.detect_markers(img)
        if ids and 16 in ids: 
            print("Found ID 16!!!!")
            turnInPlace(33, dir=0) 
            driveXMeters(0.2, 0.2)
            driveUntilLine(500)

        else:
            print("Did NOT find ID 16, trying again.... ")
            turnInPlace(59, dir=0)
            ok, new_img, _ = cam.getImage()
            if ok: 
                self.start_looking_for_ID16(img)

    def release(self):
        cv.destroyAllWindows()

aru = ArucoDetector()





"""""
    def DONT_orient_and_turn_to_B(self, frame, ids, centers):
        if ids is not None: 
            ids_flat = ids #first ID it sees 
            print(f"Detected IDs: {ids_flat}")

            if 12 in ids_flat and 13 in ids_flat: 
                print("Seeing both markers, assume centered")
                driveXMeters(0.3, 0.2)
                turnInPlace(60,dir=1) #turn 90 degrees right, 63 is 90
            elif 12 in ids_flat:
                print("Seeing marker 12, turning 90+ degrees")  
                driveXMeters(0.3, 0.2)
                turnInPlace(68,dir=1) #turn more than 90 degrees right 
            elif 13 in ids_flat:
                print("Seeing marker 13, turning less than 90 degrees")
                driveXMeters(0.3, 0.2)
                turnInPlace(45,dir=1)
            else: 
                print("Seeing neither marker, turning a bit")
                turnInPlace(23,dir=1)

    def drive_to_line_and_find_C(self):
        #print("Driving to line")
        driveXMeters(0.6, 0.2)
        driveUntilLine() 
        print("Line reached, now turning left to look for C")
        while True: 
            turnInPlace(deg=30, dir=0, ang_speed=0.5)  # Clockwise, small step
            #service.send(service.topicCmd + "ti/rc", "0 0.3")
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break
            ids, corners, centers, angles, img = self.detect_markers(img)
            if ids:
                if 14 and 15 in ids:
                    #turnInPlace(deg = 20, dir=0)
                    print("Found ID 14 and 15 marker, stop turning")
                    driveXMeters(0.2, 0.2) 
                    break
        print("Now driving to the next line")
        driveUntilLine()
        print("Second line reached")
        #turnInPlace(63, dir=0) #90 degrees left 

    def find_and_orient_to_BBBBB(self, img):
        while True: 
            ids, corners, centers, angles, img = self.detect_markers(img)
            if ids: 
                if 13 in ids:
                    print("Found ID 13 markers")
                    #turrnInPlace(deg=30, dir=0, ang_speed=0.5)  # Clockwise, small step
                    service.send(service.topicCmd + "ti/rc", "0 -0.25")
                    #self.orient_and_turn_to_B(img, ids, centers)
                    if 13 not in ids: 
                        break
                else: 
                    print("No ID 13 marker found, turning a bit:-)")
                    #turnInPlace(deg=15, dir=0, ang_speed=0.5)  # Clockwise, small step
                    service.send(service.topicCmd + "ti/rc", "0 -0.25") #here i want it to turn a little right and a little left instead to look for
            else: 
                print("No B marker found, turning a bit:-)")
                #turnInPlace(deg=15, dir=0, ang_speed=0.5)  # Clockwise, small step
                service.send(service.topicCmd + "ti/rc", "0 -0.25")  # rotate right
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break
        driveUntilLine()
"""
    
#right after picking up the ball, trigger sensor to see 
#when it sees second line, turn left 90 degrees 
#drive forward 30 centimeters or something
#drop the ball, lift arm
#turn 90 to the right 
#find line again
#turn left until aruco is centered
#when centered, trigger sensor 
#or just follow the line until sensor says its close enough 
