import time
import sys
import math
from config.config import MAX_RACK_HEIGHT_MM, TICKS_PER_MM_Z_N20
from drivers.arduino_bridge import ArduinoBridge
from drivers.lidar_driver import YDLidarSafetyDriver
from perception.stereo_avoidance import StereoAvoidanceSystem
from perception.camera_scanner import ArducamScanner
from perception.inventory_manager import InventoryManager
from navigation.path_planner import AStarPlanner
from navigation.motion_controller import MotionController
from navigation.slam_mapper import WarehouseMapper
from navigation.route_optimizer import RouteOptimizer

DOCKING_POSITION = (0.0, 0.0)

def run_z_axis_scan(rack_id, bridge, camera, db):
    print(f"\n[*] Scanning {rack_id}...")
    target_ticks = int(MAX_RACK_HEIGHT_MM * TICKS_PER_MM_Z_N20)
    bridge.send_drive_command(0, 0, target_ticks)
    
    start_time = time.time()
    seen_qrs = set()
    
    while bridge.get_z_height_mm() < MAX_RACK_HEIGHT_MM - 20: 
        if time.time() - start_time > 180: break
            
        current_z = bridge.get_z_height_mm()
        qr_list, frame = camera.scan_frame()
        for qr in qr_list:
            if qr not in seen_qrs:
                seen_qrs.add(qr)
                db.record_item(rack_id, current_z, qr, frame)
                print(f"    [+] Logged: {qr} at {current_z}mm")

    bridge.send_drive_command(0, 0, 0)
    while bridge.get_z_height_mm() > 50: time.sleep(0.1)

def navigate_with_replanning(target_pos, bridge, planner, controller, lidar, stereo):
    while True:
        if lidar.is_obstacle_ahead() or stereo.check_for_3d_obstacles():
            print("[!] Fused Safety Check failed. Waiting for clearance...")
            time.sleep(1)
            continue
            
        path = planner.plan(bridge.get_pose()[:2], target_pos)
        if not path:
            print("[!] CRITICAL: No viable path exists.")
            return False
            
        status = controller.navigate_path(path, lidar)
        if status == "REACHED": return True
        elif status == "BLOCKED":
            curr_x, curr_y, curr_theta = bridge.get_pose()
            obs_x = curr_x + 0.4 * math.cos(curr_theta)
            obs_y = curr_y + 0.4 * math.sin(curr_theta)
            planner.set_obstacle(obs_x, obs_y, radius_m=0.3)

def main():
    bridge = ArduinoBridge()
    lidar = YDLidarSafetyDriver()
    stereo = StereoAvoidanceSystem()
    camera = ArducamScanner()
    db = InventoryManager()
    planner = AStarPlanner()
    controller = MotionController(bridge)
    
    mapper = WarehouseMapper(lidar, planner, bridge)
    optimizer = RouteOptimizer(dock_pos=DOCKING_POSITION)
    
    print("\n--- ZOMATO/BLINKIT AUTONOMOUS WAREHOUSE SYSTEM ---")
    
    try:
        rack_database = mapper.roam_and_map(roaming_duration_sec=60)
        if not rack_database: sys.exit(1)
            
        print("\n[*] SLAM complete. Navigating back to Docking Station (0,0)...")
        navigate_with_replanning(DOCKING_POSITION, bridge, planner, controller, lidar, stereo)
            
        while True:
            print("\n[ DOCKED AND WAITING ]")
            query = input("Enter Rack IDs to scan (comma separated) or 'exit': ").strip()
            if query.lower() == 'exit': break
                
            queried_racks = [r.strip() for r in query.split(',')]
            optimal_sequence = optimizer.get_perfect_path(bridge.get_pose()[:2], queried_racks, rack_database)
            
            if not optimal_sequence: continue
                
            for rack_id in optimal_sequence:
                print(f"\n[>>>] Dispatching to {rack_id}")
                if navigate_with_replanning(rack_database[rack_id], bridge, planner, controller, lidar, stereo):
                    run_z_axis_scan(rack_id, bridge, camera, db)
                    
            print("\n[*] Queries resolved. Returning to Docking Station...")
            navigate_with_replanning(DOCKING_POSITION, bridge, planner, controller, lidar, stereo)

    except KeyboardInterrupt:
        bridge.trigger_estop()

    finally:
        bridge.close()
        lidar.stop()
        stereo.release()
        camera.release()
        sys.exit(0)

if __name__ == "__main__":
    main()