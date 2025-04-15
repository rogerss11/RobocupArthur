
import cv2 as cv
from datetime import *
from setproctitle import setproctitle

# robot function
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service


def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()

    if not ok:  # size(img) == 0
      if cam.imageFailCnt < 5:
        print("Failed to get image.")
      return None  

    h, w, ch = img.shape
    if not service.args.silent:
      pass 
    edge.paint(img)
    if not gpio.onPi:
      cv.imshow('frame for analysis', img)
      cv.waitKey(1)
    return img