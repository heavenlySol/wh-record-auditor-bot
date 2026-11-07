import math
import time
from config.config import TOLERANCE_XY_M

class MotionController:
    def __init__(self, bridge):
        self.bridge = bridge

    def navigate_path(self, path, lidar_driver, max_linear_vel_ticks=40):
        for waypoint in path:
            obstacle_wait_start = 0
            
            while True:
                if lidar_driver.is_obstacle_ahead():
                    self.bridge.send_drive_command(0, 0, self.bridge.enc_z)
                    if obstacle_wait_start == 0: obstacle_wait_start = time.time()
                    if time.time() - obstacle_wait_start > 3.0: return "BLOCKED"
                    time.sleep(0.1)
                    continue
                else: obstacle_wait_start = 0 

                curr_x, curr_y, curr_theta = self.bridge.get_pose()
                dx = waypoint[0] - curr_x
                dy = waypoint[1] - curr_y
                dist = math.hypot(dx, dy)

                if dist < TOLERANCE_XY_M: break 

                angle_err = math.atan2(dy, dx) - curr_theta
                while angle_err > math.pi:  angle_err -= 2 * math.pi
                while angle_err < -math.pi: angle_err += 2 * math.pi

                linear_vel = max_linear_vel_ticks * max(0.0, math.cos(angle_err))
                angular_vel = 25 * angle_err 

                target_v_left = int(linear_vel - angular_vel)
                target_v_right = int(linear_vel + angular_vel)

                self.bridge.send_drive_command(target_v_left, target_v_right, self.bridge.enc_z)
                time.sleep(0.05)

        self.bridge.send_drive_command(0, 0, self.bridge.enc_z)
        return "REACHED"