# Physical Rover Constants
CHASSIS_TRACK_WIDTH_M = 0.38
WHEEL_RADIUS_M = 0.045
TICKS_PER_REV_PG3655B = 2100      
TICKS_PER_MM_Z_N20 = 25.5         

# Ports
SERIAL_PORT = "/dev/ttyACM0"
YDLIDAR_PORT = "/dev/ttyUSB0"

# Cameras
SCANNING_CAMERA_INDEX = 0         
STEREO_CAMERA_INDEX = 1           

# Navigation & Safety Constraints
OBSTACLE_SAFETY_DISTANCE_M = 5.0  
CRITICAL_STOP_DISTANCE_M = 0.35
MAX_RACK_HEIGHT_MM = 1800.0       
TOLERANCE_XY_M = 0.10             
TOLERANCE_Z_MM = 20.0             

# Database
DB_PATH = "warehouse_inventory.db"
CSV_LOG_PATH = "inventory_audit.csv"