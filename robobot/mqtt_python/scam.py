import numpy as np
import cv2 as cv
from threading import Thread
import time as t
from datetime import *

class SCam:
    def __init__(self):
        self.cap = None
        self.th = None
        self.savedFrame = None
        self.frameTime = datetime.now()
        self.getFrame = True
        self.cnt = 0
        self.gray = None
        self.useCam = True
        self.imageFailCnt = 0
        self.cameraAvailable = False

    def setup(self):
        if not self.useCam:
            print("% SCam:: Camera disabled (useCam=False)")
            return

        from uservice import service
        url = f'http://{service.host}:7123/stream.mjpg'
        print(f"% SCam:: Trying to open stream at {url}")
        self.cap = cv.VideoCapture(url)

        if not self.cap.isOpened():
            print("% SCam:: Failed to open MJPEG stream — trying local webcam...")
            self.cap = cv.VideoCapture(0)

        if self.cap.isOpened():
            self.cameraAvailable = True
            print("% SCam:: Camera opened successfully")
            self.th = Thread(target=self.run, daemon=True)
            self.th.start()
        else:
            self.cameraAvailable = False
            print("% SCam:: No camera available — continuing without camera")

    def getImage(self):
        if not self.cameraAvailable:
            if self.imageFailCnt == 0:
                print("% SCam:: No camera available")
            self.imageFailCnt += 1
            return False, self.savedFrame, self.frameTime

        from uservice import service
        self.getFrame = True
        cnt = 0
        while self.getFrame and cnt < 100 and not service.stop:
            t.sleep(0.01)
            cnt += 1

        if self.getFrame:
            self.imageFailCnt += 1
            return False, self.savedFrame, self.frameTime
        else:
            self.imageFailCnt = 0
            return True, self.savedFrame, self.frameTime

    def run(self):
        from uservice import service
        print("% SCam:: Camera thread running")
        cnt = 0
        first = True
        while self.cap and self.cap.isOpened() and not service.stop:
            if self.getFrame or first:
                ret, frame = self.cap.read()
                self.frameTime = datetime.now()
                if ret:
                    self.savedFrame = frame
                    self.getFrame = False
                    self.cnt += 1
                    if first:
                        h, w, ch = self.savedFrame.shape
                        print(f"% SCam:: Camera active: {w}x{h} with {ch} channels")
                        first = False
                else:
                    print("% SCam:: Failed to read frame — ending thread")
                    break
            else:
                self.cap.read()  # discard unused frames
        print("% SCam:: Camera thread stopped")

    def terminate(self):
        if self.th is not None:
            try:
                self.th.join(timeout=2)
            except:
                print("% SCam:: Failed to join camera thread")

        if self.cap and isinstance(self.cap, cv.VideoCapture):
            self.cap.release()

        cv.destroyAllWindows()
        print("% SCam:: Camera terminated")

# create instance of this class
cam = SCam()
