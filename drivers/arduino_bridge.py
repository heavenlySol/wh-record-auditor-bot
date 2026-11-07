import serial
import threading
import time
import math
from config.config import SERIAL_PORT, WHEEL_RADIUS_M, TICKS_PER_REV_PG3655B, CHASSIS_TRACK_WIDTH_M

class ArduinoBridge:
    def __init__(self, port=SERIAL_PORT, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.lock = threading.Lock()
        self.running = True
        
        self.enc_l = 0; self.enc_r = 0; self.enc_z = 0
        self.tof_height_mm = 0
        self.is_estop = False
        
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self._prev_l = 0; self._prev_r = 0

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("DATA:"):
                    parts = line[5:].split(',')
                    if len(parts) == 5:
                        with self.lock:
                            curr_l, curr_r = int(parts[0]), int(parts[1])
                            self.enc_z = int(parts[2])
                            self.tof_height_mm = int(parts[3])
                            self.is_estop = bool(int(parts[4]))

                            dl = (curr_l - self._prev_l) * (2.0 * math.pi * WHEEL_RADIUS_M / TICKS_PER_REV_PG3655B)
                            dr = (curr_r - self._prev_r) * (2.0 * math.pi * WHEEL_RADIUS_M / TICKS_PER_REV_PG3655B)
                            self._prev_l, self._prev_r = curr_l, curr_r

                            d_center = (dl + dr) / 2.0
                            d_theta = (dr - dl) / CHASSIS_TRACK_WIDTH_M

                            self.theta += d_theta
                            self.x += d_center * math.cos(self.theta)
                            self.y += d_center * math.sin(self.theta)
            except: pass
            time.sleep(0.005)

    def send_drive_command(self, v_left, v_right, target_z_ticks):
        if not self.is_estop:
            cmd = f"CMD:{int(v_left)},{int(v_right)},{int(target_z_ticks)}\n"
            with self.lock: self.ser.write(cmd.encode('utf-8'))

    def trigger_estop(self):
        self.is_estop = True
        with self.lock:
            self.ser.write(b"ESTOP\n")
            self.ser.flush()

    def get_pose(self):
        with self.lock: return self.x, self.y, self.theta

    def get_z_height_mm(self):
        with self.lock: return self.tof_height_mm

    def close(self):
        self.running = False
        self.trigger_estop()
        self.ser.close()