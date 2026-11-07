# Warehouse Record Auditor Bot

A prototype mobile warehouse rover that maps its surroundings, accepts rack IDs to scan, reads QR codes while moving a lift along each rack, and stores an inventory audit with image evidence.

The software is intended to run on the rover computer with the matching sensors and Arduino firmware. It is hardware-dependent; starting it without the configured serial device, YDLidar, and cameras will fail. Review and adapt the configuration for your hardware before operating the rover.

## What it does

At startup, the program roams for 60 seconds while collecting YDLidar points, marks the points as obstacles in an occupancy grid, and clusters them into candidate rack locations. It then returns to the dock and waits for a comma-separated list of rack IDs. For each recognized rack, it plans a route, checks for obstacles, scans QR codes during a vertical lift pass, and records each newly seen code with the rack ID, lift height, timestamp, and image path. After each query it attempts to return to the dock.

The audit is written to both a SQLite database (`warehouse_inventory.db`) and a CSV log (`inventory_audit.csv`). Image evidence is saved under `proofs/` when a camera frame is available.

## Repository layout

| Path | Purpose |
| --- | --- |
| `main.py` | Rover startup, mapping, rack query loop, scanning, and cleanup |
| `config/config.py` | Hardware ports, camera indices, dimensions, safety distances, and output paths |
| `drivers/` | Arduino serial bridge and YDLidar driver |
| `perception/` | QR scanner, stereo obstacle check, and inventory audit storage |
| `navigation/` | Mapping, A* grid planning, motion control, and route ordering |
| `arduino_firmware/rover_firmware.ino` | Arduino motor, encoder, lift, range sensor, and emergency stop firmware |
| `3d_model/` | Rover renders and assembly model |

## Hardware and software prerequisites

- Linux computer connected to the rover's Arduino, YDLidar, QR scanning camera, and stereo camera.
- Arduino-compatible board and motor/sensor wiring matching `arduino_firmware/rover_firmware.ino`.
- Python 3.9 or newer recommended.
- OpenCV camera support, `pyzbar` and its system ZBar library, and the YDLidar Python SDK.

The default device paths and camera indices are `/dev/ttyACM0`, `/dev/ttyUSB0`, camera `0`, and camera `1`. Change these in `config/config.py` to match the host. The Arduino serial connection uses 115200 baud.

## Installation

From the repository root, create and activate a virtual environment, then install the Python requirements:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Install the system ZBar library required by `pyzbar` (package name varies by Linux distribution; on Debian/Ubuntu it is `libzbar0`). Install the [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK) and its Python bindings separately. The commented block in `requirements.txt` lists a source-build outline; it is not installed by `pip install -r requirements.txt`.

Upload `arduino_firmware/rover_firmware.ino` to the rover controller using the Arduino IDE or CLI. Ensure the host user can access the serial devices; on many Debian/Ubuntu systems this means membership in the `dialout` group followed by a new login session.

## Configuration

Edit `config/config.py` for the connected hardware and physical measurements. Important settings include:

- `SERIAL_PORT`, `YDLIDAR_PORT`: device paths.
- `SCANNING_CAMERA_INDEX`, `STEREO_CAMERA_INDEX`: camera indices as seen by OpenCV.
- `CHASSIS_TRACK_WIDTH_M`, `WHEEL_RADIUS_M`, `TICKS_PER_REV_PG3655B`: odometry calibration.
- `TICKS_PER_MM_Z_N20`, `MAX_RACK_HEIGHT_MM`: lift calibration and scan height.
- `OBSTACLE_SAFETY_DISTANCE_M`, `CRITICAL_STOP_DISTANCE_M`: lidar filtering and stop threshold.
- `DB_PATH`, `CSV_LOG_PATH`: audit output locations.

The firmware expects newline-terminated serial commands of the form `CMD:<left>,<right>,<z_ticks>`. It streams encoder/lift/range/emergency-stop readings as `DATA:<left>,<right>,<z_ticks>,<height_mm>,<estop>`. The host and firmware formats must stay in sync.

## Run

With the rover hardware connected and clear of people and obstacles, run from the repository root so that the configured relative audit paths resolve as expected:

```bash
source .venv/bin/activate
python main.py
```

After mapping and returning to the dock, enter rack IDs printed during mapping, separated by commas, for example:

```text
Rack_1,Rack_3
```

Enter `exit` at the prompt to stop. Ctrl+C triggers the host-side emergency-stop command and then closes the sensor connections.

## Audit data

Each detected QR code is recorded with these fields:

- Timestamp
- Rack ID
- Lift height in millimeters
- QR payload
- Image evidence path

Repeated QR payloads are suppressed within a single rack scan, so the audit records the first observation in that pass. The SQLite table is named `inventory_audit`; the CSV columns are `Timestamp`, `Rack_ID`, `Z_Height_mm`, `QR_Content`, and `Image_Path`.

## Current limitations

- Rack IDs and rack coordinates are inferred from point clusters during a short roaming run; they are not loaded from a pre-existing warehouse map.
- The map, planner, and pose estimate are in-memory and restart from the origin each run.
- The stereo obstacle check is a simple disparity threshold and depends on camera alignment and lighting.
- The configuration, sensor thresholds, lift behavior, and emergency-stop chain need validation on the specific physical rover before operation.
- This repository does not include a simulator, automated test suite, or a software-only mode.

## License

No license file is currently included. Contact the repository owner for reuse terms.
