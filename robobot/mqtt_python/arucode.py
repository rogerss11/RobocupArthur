import cv2 as cv
import cv2.aruco as aruco
import numpy as np

#also because we want to open the camera once, then reuse it when needed
class ArucoDetector: #creating a class for encapsulation, all arucode here and easy to add 
    def __init__ (self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #this is the dictonary
        #returns an object that contains tunable settings for marker detection, can refine them
        self.parameters = aruco.DetectorParameters_create() #the parameters for marker detection
        self.cap = cv.VideoCapture(0) #initalizes the camera

    def detect_markers(self):
        ret, frame = self.cap.read() #ret true if image captures, frame is the image
        if not ret:
            return None, None
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #converts the image to grayscale

        #basically detects the markers, returns the corners, ids, and rejected points
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None: 
            #draws the detected on the image. 
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            #convert to simple list
            return ids.flatten().tolist(), corners #returning detected ids and corner positions
        return None, None 
    
    def release(self):
        self.cap.release()
        cv.destroyAllWindows()



