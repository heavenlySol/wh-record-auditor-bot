import ydlidar
import threading
import math
import time
from config.config import YDLIDAR_PORT, OBSTACLE_SAFETY_DISTANCE_M, CRITICAL_STOP_DISTANCE_M

class YDLidarSafetyDriver:
    def __init__(self):
        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, YDLIDAR_PORT)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
        
        if self.laser.initialize(): self.laser.turnOn()
        
        self.lock = threading.Lock()
        self.latest_scan = []
        self.emergency_stop_triggered = False
        self.running = True

        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()

    def _scan_loop(self):
        scan = ydlidar.LaserScan()
        while self.running and ydlidar.os_isOk():
            if self.laser.doProcessSimple(scan):
                filtered_scan = []
                danger = False
                for point in scan.points:
                    dist_m = point.range
                    angle = math.degrees(point.angle)
                    
                    if 0 < dist_m <= OBSTACLE_SAFETY_DISTANCE_M:
                        filtered_scan.append((angle, dist_m))
                        if (angle < 30 or angle > 330) and dist_m < CRITICAL_STOP_DISTANCE_M:
                            danger = True

                with self.lock:
                    self.latest_scan = filtered_scan
                    self.emergency_stop_triggered = danger
            time.sleep(0.05)

    def is_obstacle_ahead(self):
        with self.lock: return self.emergency_stop_triggered

    def get_latest_points_xy(self):
        with self.lock:
            points = []
            for angle, dist in self.latest_scan:
                rad = math.radians(angle)
                points.append((dist * math.cos(rad), dist * math.sin(rad)))
            return points

    def stop(self):
        self.running = False
        self.laser.turnOff()
        self.laser.disconnecting()