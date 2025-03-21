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
            print(f"[DEBUG] Frame shape: {frame.shape}")
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            print("greyscale ok")
            print("[DEBUG] OpenCV version:", cv.__version__)

            corners, ids, rejected = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

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

