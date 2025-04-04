import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle

# robot function
from scam import cam
from sedge import edge
from sgpio import gpio
from uservice import service

setproctitle("calibration-capture")

############################################################

def imageAnalysis(save, img_num):
    if cam.useCam:
        ok, img, imgTime = cam.getImage()

        if not ok:
            if cam.imageFailCnt < 5:
                print("Failed to get image.")
            return None

        if save:
            fn = f"NEWcalib_image_{img_num:02d}.jpg"
            cv.imwrite(fn, img)
            print(f"% Saved image {fn}")
        return img

############################################################

def loop():
    img_num = 0

    print("% Calibration mode: Taking 1 image every 30 seconds (Ctrl+C to stop)")

    while True:
        img = imageAnalysis(True, img_num)
        img_num += 1
        t.sleep(10)

############################################################

if __name__ == "__main__":
    print("% Starting")
    service.setup('localhost')
    if service.connected:
        loop()
        service.terminate()
    print("% Main Terminated")
