

import cv2 as cv
import numpy as np
from scam import cam  # Use the robot's existing camera system

class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)  # Dictionary
        self.parameters = cv.aruco.DetectorParameters()

    def detect_markers(self):
        """
        Gets a frame from the robot's camera and detects ArUco markers.
        """
        ok, frame, _ = cam.getImage()  # Get the latest image from the robot's camera

        if not ok or frame is None or frame.size == 0:  # Ensure valid frame
            print("⚠️ Warning: Failed to capture image from camera.")
            return None, None, None

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert to grayscale
        corners, ids, rejected = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                position = corners[i][0].tolist()
                cv.putText(frame, f"id={marker_id}", tuple(int(x) for x in position[0]), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            return ids.flatten().tolist(), corners, frame
        else:
            return None, None, frame

    def release(self):
        """ Releases OpenCV windows (no need to release the camera). """
        cv.destroyAllWindows()
