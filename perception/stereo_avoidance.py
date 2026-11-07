import cv2
import numpy as np
from config.config import STEREO_CAMERA_INDEX

class StereoAvoidanceSystem:
    def __init__(self):
        self.cap = cv2.VideoCapture(STEREO_CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

    def check_for_3d_obstacles(self):
        ret, frame = self.cap.read()
        if not ret: return False
            
        height, width, _ = frame.shape
        gray_l = cv2.cvtColor(frame[0:height, 0:width//2], cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(frame[0:height, width//2:width], cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(gray_l, gray_r)
        roi = disparity[height//3:height, width//4:width//4*3]
        
        if np.sum(roi > 32) > 5000: return True 
        return False

    def release(self):
        self.cap.release()