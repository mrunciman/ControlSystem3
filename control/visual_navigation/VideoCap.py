import cv2
import numpy as np
import os.path
import os 
from datetime import datetime
import time

class CameraSource:
    def __init__(self, cam_idx, width, height):
        self.cap = cv2.VideoCapture(cam_idx)
        self.width = width
        self.height = height
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def initialize(self):
        if not self.cap.isOpened():
            return False
            # raise IOError("Cannot open left webcam")
        else:
            return True

    def receive_img(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            return None
        return frame
        
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()