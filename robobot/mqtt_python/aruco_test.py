import cv2 as cv
import numpy as np
from eva_drive import turnInPlace, driveXMeters, driveUntilLine
from scam import cam
from uservice import service
from sedge import edge 
from uservice import service
from spose import pose
import time as t



class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        #self.parameters = cv.aruco.DetectorParameters_create()
        self.parameters = cv.aruco.DetectorParameters()

        # --- Load calibration ---
        calib_data = np.load("calibration_data.npz")
        self.mtx = calib_data['mtx']
        self.dist = calib_data['dist']

    def detect_markers(self, frame):
        if frame is None or frame.size == 0:
            print("Warning: Invalid image received.")
            return None, None, None, None, frame

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        #print("corners:", corners)
        #print("ids:", ids)

        centers = []
        angles = []

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

            # Get real distance using calibration
            marker_length = 0.18  # marker side length in meters (18 cm)
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, marker_length, self.mtx, self.dist
            )
            
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

                # --- Get distance ---
                #tvec = tvecs[i][0]
                #print(tvecs[i])
                #distance = np.linalg.norm(tvec) * 100  # to cm
                cv.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)  # green dot
                print(f"ID: {marker_id} | Center: ({center_x}, {center_y}) | Angle: {angle:.2f} deg")

            return ids.flatten().tolist(), corners, centers, angles, frame
        else:
            return None, None, None, None, frame
        
    def turn_left(self, img):
        driveXMeters(x=-0.2, vel=0.2)
        service.send(service.topicCmd + "T0/servo", "1 -150 0")
        #turnInPlace(deg=30, dir=0, ang_speed=0.5)
        #driveXMeters(x=0.2, speed=0.2)
        #turnInPlace(deg=20, dir=0, ang_speed=0.5)
        turnInPlace(deg=45, dir=0, ang_speed=0.5)  # Clockwise, small step

        print("% Doing arc turn")
        #service.send(service.topicCmd + "T0/leds", "16 100 100 0")  # yellowish LED

        # Turn with forward speed + angular velocity
        forward_speed = 0.2   # m/s
        turn_rate = -0.3      # rad/s (negative = clockwise)

        pose.tripBreset()
        state = 0
        while not service.stop:
            if state == 0:
                service.send("robobot/cmd/ti/rc", f"{forward_speed} {turn_rate}")
                state = 1
            elif state == 1:
                if pose.tripBtimePassed() > 2.5:  # adjust timing based on how big arc you want
                    service.send("robobot/cmd/ti/rc", "0.0 0.0")
                    state = 99
            elif state == 99:
                print(f"# Finished arc in {pose.tripBtimePassed():.2f} sec")
                break
            t.sleep(0.05)

        #service.send(service.topicCmd + "T0/leds", "16 0 0 0")  # turn off LEDs

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

        
    def find_and_orient_to_B(self, img):
        found_13 = False
        turning_left = True

        print("🔍 Looking for ID 13...")

        while not found_13:
            # Turn slowly left or right
            #turn_rate = -0.3 if turning_left else 0.3
            #turn_rate = 0.4
            #service.send(service.topicCmd + "ti/rc", f"0 {turn_rate}")
            turnInPlace(deg=30, dir=1, ang_speed=0.5)  # Clockwise, small step
            #turnInPlace(deg=50, dir=1, ang_speed=0.5)  # Counter-clockwise, small step
        
            # Get image and check
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break

            ids, _, _, _, img = self.detect_markers(img)
            if ids:
                ids_flat = ids
                print(f"Detected: {ids_flat}")
                if 13 in ids_flat:
                    print("✅ Found ID 13!")
                    service.send(service.topicCmd + "ti/rc", "0 0")  # stop turning
                    found_13 = True
                    break

            # Alternate turn direction
            #turning_left = not turning_left
            t.sleep(0.3)  # Pause briefly before switching direction

        # Step 2: Turn left until ID 13 is no longer visible
        

        while True:
            ok, img, _ = cam.getImage()
            if not ok:
                print("No image from camera.")
                break

            ids, _, _, _, img = self.detect_markers(img)
            if ids:
                ids_flat = ids
                print(f"Still detecting: {ids_flat}")
                print("🔄 Turning left until ID 13 disappears...")
                service.send(service.topicCmd + "ti/rc", "0 0.25")  # Slow turn left
                if 13 not in ids_flat:
                    service.send(service.topicCmd + "ti/rc", "0 0")
                    print("❌ ID 13 gone — stop turning") #
                    break
            else:
                print("No marker visible — stop turning")
                break

            t.sleep(0.1)

        # Final step
        print("turn a little to the left still")
        turnInPlace(15,0)
        print("🚗 Driving until line")
        driveUntilLine()

        
    def orient_and_turn_to_B(self, frame, ids, centers):
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

    def release(self):
        cv.destroyAllWindows()

#right after picking up the ball, trigger sensor to see 
#when it sees second line, turn left 90 degrees 
#drive forward 30 centimeters or something
#drop the ball, lift arm
#turn 90 to the right 
#find line again
#turn left until aruco is centered
#when centered, trigger sensor 
#or just follow the line until sensor says its close enough 
