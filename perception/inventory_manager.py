import sqlite3
import csv
import os
import datetime
from config.config import DB_PATH, CSV_LOG_PATH

class InventoryManager:
    def __init__(self):
        self.db_path = DB_PATH
        self.csv_path = CSV_LOG_PATH
        self._init_db()
        self._init_csv()

    def _init_db(self):
        conn = sqlite3.connect(self.db_path)
        conn.cursor().execute('''
            CREATE TABLE IF NOT EXISTS inventory_audit (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT, rack_id TEXT, z_height_mm REAL, 
                qr_content TEXT, image_proof_path TEXT
            )
        ''')
        conn.commit()
        conn.close()

    def _init_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                csv.writer(f).writerow(["Timestamp", "Rack_ID", "Z_Height_mm", "QR_Content", "Image_Path"])

    def record_item(self, rack_id, z_height_mm, qr_content, frame=None):
        timestamp = datetime.datetime.now().isoformat()
        img_filename = f"proofs/{rack_id}_{int(z_height_mm)}mm_{int(datetime.datetime.now().timestamp())}.jpg"

        if frame is not None:
            os.makedirs("proofs", exist_ok=True)
            import cv2
            cv2.imwrite(img_filename, frame)

        conn = sqlite3.connect(self.db_path)
        conn.cursor().execute('''
            INSERT INTO inventory_audit (timestamp, rack_id, z_height_mm, qr_content, image_proof_path)
            VALUES (?, ?, ?, ?, ?)
        ''', (timestamp, rack_id, z_height_mm, qr_content, img_filename))
        conn.commit()
        conn.close()

        with open(self.csv_path, mode='a', newline='') as f:
            csv.writer(f).writerow([timestamp, rack_id, z_height_mm, qr_content, img_filename])