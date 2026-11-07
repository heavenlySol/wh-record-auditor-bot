import time
import math
import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage

class WarehouseMapper:
    def __init__(self, lidar, planner, bridge):
        self.lidar = lidar
        self.planner = planner
        self.bridge = bridge
        self.rack_database = {}
        self.global_map_points = []

    def _lidar_to_global(self, local_points, pose_x, pose_y, pose_theta):
        global_pts = []
        for (local_x, local_y) in local_points:
            gx = pose_x + (local_x * math.cos(pose_theta) - local_y * math.sin(pose_theta))
            gy = pose_y + (local_x * math.sin(pose_theta) + local_y * math.cos(pose_theta))
            global_pts.append((gx, gy))
        return global_pts

    def roam_and_map(self, roaming_duration_sec=60):
        print(f"[*] Commencing Autonomous Roaming SLAM for {roaming_duration_sec} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < roaming_duration_sec:
            curr_x, curr_y, curr_theta = self.bridge.get_pose()
            local_pts = self.lidar.get_latest_points_xy()
            global_pts = self._lidar_to_global(local_pts, curr_x, curr_y, curr_theta)
            
            for gx, gy in global_pts:
                self.global_map_points.append((gx, gy))
                self.planner.set_obstacle(gx, gy, radius_m=0.2) 

            if self.lidar.is_obstacle_ahead():
                self.bridge.send_drive_command(0, 0, self.bridge.enc_z)
                time.sleep(0.5)
                self.bridge.send_drive_command(-40, 40, self.bridge.enc_z) 
                time.sleep(1.0) 
            else:
                self.bridge.send_drive_command(40, 40, self.bridge.enc_z)
                
            time.sleep(0.1)

        self.bridge.send_drive_command(0, 0, self.bridge.enc_z)
        return self._cluster_racks()

    def _cluster_racks(self):
        if not self.global_map_points: return {}

        pts_array = np.array(self.global_map_points)
        Z = linkage(pts_array, method='ward')
        clusters = fcluster(Z, t=1.0, criterion='distance')

        unique_clusters = set(clusters)
        rack_id_counter = 1

        for cid in unique_clusters:
            cluster_points = pts_array[clusters == cid]
            cx, cy = np.mean(cluster_points[:, 0]), np.mean(cluster_points[:, 1])
            target_x, target_y = cx - 0.5, cy
            
            rack_name = f"Rack_{rack_id_counter}"
            self.rack_database[rack_name] = (target_x, target_y)
            print(f"[+] Identified {rack_name} at Map Coordinate: ({target_x:.2f}, {target_y:.2f})")
            rack_id_counter += 1

        return self.rack_database