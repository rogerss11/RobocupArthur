import cv2 as cv
import numpy as np
from eva_drive import turnInPlace, driveXMeters


class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters_create()

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
        print("corners:", corners)
        print("ids:", ids)


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
        
    def orient_and_turn_to_B(self, frame, ids, centers):
        while ids is None:
            turnInPlace()
            print("No marker detected")
            return

        ids_flat = [int(id[0]) for id in ids] #first ID it sees 
        print(f"Detected IDs: {ids_flat}")

        if 12 in ids_flat and 13 in ids_flat: 
            print("Seeing both markers, assume centered")
            turnInPlace(63,dir=1) #turn 90 degrees right, 63 is 90
        elif 12 in ids_flat:
            print("Seeing marker 12, turning 90+ degrees")  
            turnInPlace(93,dir=1) #turn more than 90 degrees right 
        elif 13 in ids_flat:
            print("Seeing marker 13, turning less than 90 degrees")
            turnInPlace(33,dir=-1)
        else: 
            #turninplace 
            print("No B markers detected")

    def release(self):
        cv.destroyAllWindows()


"""
import cv2 as cv
import numpy as np

class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters_create()  # <- safer method


    def detect_markers(self, frame):
        if frame is None or frame.size == 0:
            print("Warning: Invalid image received.")
            return None, None, None

        try:
            #print(f"[DEBUG] Frame shape: {frame.shape}")
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            #print("greyscale ok")
            #print("[DEBUG] OpenCV version:", cv.__version__)

            corners, ids, rejected = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            centers = []
            angles = []

            if ids is not None:
                try:
                    # draw markers
                    cv.aruco.drawDetectedMarkers(frame, corners, ids)
                    for i, marker_id in enumerate(ids.flatten()):
                        if len(corners[i][0]) > 0:
                            pt = tuple(map(int, corners[i][0][0]))
                            cv.putText(frame, f"id={marker_id}", pt,
                                    cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                except Exception as draw_err:
                    print(f"[DRAW ERROR] {draw_err}")
                return ids.flatten().tolist(), corners, frame
            else:
                print("No markers found.")
                return None, None, frame

        except Exception as e:
            print(f"Error in ArUco Detection: {e}")
            return None, None, None
    def release(self):
        cv.destroyAllWindows()

    def turning(self, ids, centers, angles):
        #center coordinate, center angles
        image_center = (400, 300)
        center_tolerance = 30 #pixels
        angle_tolerance = 5 #degrees

        dx = center[0] - image_center[0] #positive, too far right
        dy = center[1] - image_center[1] #positive, too far down
        print(f"Marker center offset: dx={dx}, dy={dy}, angle={angle:.2f}")
    
    
        # Adjust heading based on position
        if abs(dx) > CENTER_TOLERANCE:
            if dx > 0:
                print("Marker is to the right → turn right")
                service.send(service.topicCmd + "ti/rc", "0 0.2")  # rotate right
            else:
                print("Marker is to the left → turn left")
                service.send(service.topicCmd + "ti/rc", "0 -0.2")  # rotate left

        # Adjust heading based on angle
        elif abs(angle) > ANGLE_TOLERANCE:
            if angle > 0:
                print("Marker angled to the right → turn right slightly")
                service.send(service.topicCmd + "ti/rc", "0 0.1")
            else:
                print("Marker angled to the left → turn left slightly")
                service.send(service.topicCmd + "ti/rc", "0 -0.1")

        else:
            print("Marker centered and straight → stop")
            service.send(service.topicCmd + "ti/rc", "0 0")
            state = 99

"""