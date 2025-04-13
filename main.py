import threading
import time
from datetime import datetime
import sys

sys.path.append("/home/pi/mss")
from GPS.GpsRtkClient import GPSrtk  # Zaimportuj klasę GPSrtk z GPSrtk.py
from IMU.ImuReader import IMUReader  # import board

import board
import busio
import math
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C


# ---------- Funkcja IMU ----------
def run_imu(stop_event):
    # import board  # Przeniesione tu, żeby uniknąć błędu, gdy IMU wyłączone

    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)
    time.sleep(1)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

    def quaternion_to_euler(w, x, y, z):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, +1.0), -1.0)
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    interval = 0.1  # 100 ms = 10 Hz
    last_time = time.perf_counter()
    
    while not stop_event.is_set():
        now = time.perf_counter()
        if now - last_time >= interval:
            last_time = now
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            try:
                accel_x, accel_y, accel_z = bno.acceleration
                lin_accel_x, lin_accel_y, lin_accel_z = bno.linear_acceleration
                gyro_x, gyro_y, gyro_z = bno.gyro
                mag_x, mag_y, mag_z = bno.magnetic
                quat_i, quat_j, quat_k, quat_real = bno.quaternion
                roll, pitch, yaw = quaternion_to_euler(quat_real, quat_i, quat_j, quat_k)

                print(
                    "%s | X: %.3f Y: %.3f Z: %.3f m/s^2 | X: %.3f Y: %.3f Z: %.3f m/s^2 | "
                    "X: %.3f Y: %.3f Z: %.3f rads/s | X: %.3f Y: %.3f Z: %.3f uT | "
                    "I: %.3f J: %.3f K: %.3f Real: %.3f | R: %.3f P: %.3f Y: %.3f" % (
                        timestamp,
                        accel_x, accel_y, accel_z,
                        lin_accel_x, lin_accel_y, lin_accel_z,
                        gyro_x, gyro_y, gyro_z,
                        mag_x, mag_y, mag_z,
                        quat_i, quat_j, quat_k, quat_real,
                        roll, pitch, yaw
                    )
                )

            except Exception as e:
                print(f"Błąd odczytu z IMU: {e}")


# ---------- Funkcja GPS ----------
def run_gps(stop_event):
    gps = GPSrtk(
        serial_port='/dev/ttyUSB0',
        baudrate=115200,
        caster='system.asgeupos.pl',
        port=8080,
        mountpoint='/RTN4G_VRS_RTCM32',
        user='pwmgr/adamwrb:Globus7142001',
        save_file=False
    )

    if not gps.connect_ntrip():
        return
    if not gps.connect_gnss():
        return

    while not stop_event.is_set():
        try:
            raw_data, parsed_data = gps.nmr.read()
            if raw_data is None:
                continue
            if b"GNGGA" in raw_data:
                RTCM_response = gps.send_gga_to_ntrip(raw_data.decode())
                gps.serial_com.write(RTCM_response)
                gps.GGAdata = parsed_data
            if b"GNVTG" in raw_data:
                gps.VTGdata = parsed_data
                gps.print_data()
        except Exception as e:
            print(f"Błąd w pętli GPS: {e}")


# ---------- Główna część ----------
if __name__ == "__main__":
    stop_event = threading.Event()

    try:
        # GPS i IMU jako wątki
        gps_thread = threading.Thread(target=run_gps, args=(stop_event,))
        imu_thread = threading.Thread(target=run_imu, args=(stop_event,))  # Możesz zakomentować, jeśli nie chcesz

        gps_thread.start()
        imu_thread.start()

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nZatrzymywanie programu...")
        stop_event.set()
        gps_thread.join()
        imu_thread.join()
        print("Zamknięto poprawnie.")
