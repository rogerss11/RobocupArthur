import cv2 as cv
import numpy as np

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
                tvec = tvecs[i][0]
                print(tvecs[i])
                distance = np.linalg.norm(tvec) * 100  # to cm

                print(f"ID: {marker_id} | Center: ({center_x}, {center_y}) | Angle: {angle:.2f} deg | Distance: {distance:.2f} cm")


            return ids.flatten().tolist(), corners, centers, angles, frame
        else:
            return None, None, None, None, frame

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

"""