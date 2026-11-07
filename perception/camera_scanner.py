import cv2
import numpy as np
from pyzbar.pyzbar import decode
from config.config import SCANNING_CAMERA_INDEX

class ArducamScanner:
    def __init__(self):
        self.cap = cv2.VideoCapture(SCANNING_CAMERA_INDEX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1) 
        self.clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

    def scan_frame(self):
        ret, frame = self.cap.read()
        if not ret: return [], None

        enhanced = self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        decoded_objects = decode(enhanced)
        
        results = []
        for obj in decoded_objects:
            results.append(obj.data.decode('utf-8'))
            pts = [(p.x, p.y) for p in obj.polygon] if len(obj.polygon) == 4 else []
            if pts: cv2.polylines(frame, [np.array(pts, dtype=np.int32)], True, (0, 255, 0), 3)

        return results, frame

    def release(self):
        self.cap.release()