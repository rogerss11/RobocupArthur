# calibration_from_images.py
import cv2 as cv
import numpy as np
import glob

# ---- CONFIG ----
# Adjust these if needed
checkerboard_size = (10, 7)   # inner corners (columns, rows)
square_size = 25  # mm

# Image folder (adjust if needed)
image_folder = "./"  # assumes images are in the same directory as this script

# ---- PREPARE OBJECT POINTS ----REAL LIFE
objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store points
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# ---- LOAD IMAGES ----
images = glob.glob(f"{image_folder}/*.jpg")

print(f"Found {len(images)} images for calibration.")

# ---- FIND CORNERS ----
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                   (cv.TermCriteria_EPS + cv.TermCriteria_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        # Draw and display
        cv.drawChessboardCorners(img, checkerboard_size, corners2, ret)
        cv.imshow('Detected corners', img)
        cv.waitKey(500)
    else:
        print(f"Checkerboard not found in {fname}")

cv.destroyAllWindows()

# ---- CALIBRATION ----
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\nCalibration successful.")
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist.ravel())

# ---- SAVE RESULT ----
np.savez("calibration_data.npz", mtx=mtx, dist=dist)

print("\nSaved calibration as 'calibration_data.npz'")
