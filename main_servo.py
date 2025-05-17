import threading
import time
from datetime import datetime
import os
import sys
import csv
import math

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_servokit import ServoKit  # <--- nowa biblioteka do serwa

sys.path.append("/home/pi/mss")
from GPS.GpsRtkClient import GPSrtk
from IMU.ImuReader import IMUReader


# ---------- Funkcja IMU ----------
def run_imu(stop_event):
    try:
        print("[IMU] Startuję...")

        date_str = datetime.now().strftime("%d-%m-%y")
        imu_dir = f"data/{date_str}/pi41/"
        os.makedirs(imu_dir, exist_ok=True)
        imu_file_path = os.path.join(imu_dir, "imu_data_v3.csv")

        with open(imu_file_path, mode='w', newline='') as imu_file:
            imu_writer = csv.writer(imu_file)
            imu_writer.writerow([
                "Timestamp",
                "Accel_X", "Accel_Y", "Accel_Z",
                "LinAccel_X", "LinAccel_Y", "LinAccel_Z",
                "Gyro_X", "Gyro_Y", "Gyro_Z",
                "Mag_X", "Mag_Y", "Mag_Z",
                "Quat_I", "Quat_J", "Quat_K", "Quat_Real",
                "Roll", "Pitch", "Yaw"
            ])

            print("[IMU] Inicjalizacja I2C...")
            i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            print("[IMU] Tworzenie obiektu BNO08X...")
            bno = BNO08X_I2C(i2c)
            time.sleep(1)

            print("[IMU] Włączanie funkcji...")
            bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            bno.enable_feature(BNO_REPORT_GYROSCOPE)
            bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

            print("[IMU] Startuję pętlę pomiarową...")

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

            interval = 0.1
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

                        imu_writer.writerow([
                            timestamp,
                            f"{accel_x:.4f}", f"{accel_y:.4f}", f"{accel_z:.4f}",
                            f"{lin_accel_x:.4f}", f"{lin_accel_y:.4f}", f"{lin_accel_z:.4f}",
                            f"{gyro_x:.4f}", f"{gyro_y:.4f}", f"{gyro_z:.4f}",
                            f"{mag_x:.4f}", f"{mag_y:.4f}", f"{mag_z:.4f}",
                            f"{quat_i:.4f}", f"{quat_j:.4f}", f"{quat_k:.4f}", f"{quat_real:.4f}",
                            f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}"
                        ])

                    except Exception as e:
                        print(f"[IMU] Błąd odczytu: {e}")
    except Exception as e:
        print(f"[IMU] Błąd inicjalizacji: {e}")


# ---------- Funkcja GPS ----------
def run_gps(stop_event):
    print("[GPS] Startuję...")
    gps_client = GPSrtk(
        serial_port='/dev/ttyUSB0',
        baudrate=115200,
        caster='system.asgeupos.pl',
        port=8080,
        mountpoint='/RTN4G_VRS_RTCM32',
        user='pwmgr/adamwrb:Globus7142001',
        filename="gps_data_v3.csv",
        save_file=True,
    )
    gps_client.run(stop_event=stop_event)


# ---------- Funkcja Serwo ----------
def run_servo(stop_event):
    print("[SERVO] Startuję...")

    kit = ServoKit(channels=16)
    SERVO_CHANNEL = 0
    kit.servo[SERVO_CHANNEL].set_pulse_width_range(500, 2500)

    angle = 30
    direction = 1  # 1 = w górę, -1 = w dół
    next_change_time = time.time() + 10

    kit.servo[SERVO_CHANNEL].angle = angle
    print(f"[SERVO] Ustawiam początkowy kąt: {angle}°")

    while not stop_event.is_set():
        now = time.time()
        if now >= next_change_time:
            angle += 5 * direction

            # Sprawdzenie czy trzeba zmienić kierunek
            if angle >= 130:
                angle = 130
                direction = -1
                print("[SERVO] Osiągnięto maksymalny kąt – zawracam w dół.")
            elif angle <= 30:
                angle = 30
                direction = 1
                print("[SERVO] Osiągnięto minimalny kąt – zawracam w górę.")

            print(f"[SERVO] Ustawiam kąt: {angle}°")
            kit.servo[SERVO_CHANNEL].angle = angle
            next_change_time = now + 10

        time.sleep(0.1)  # Krótkie opóźnienie oszczędzające CPU


# ---------- Główna część ----------
if __name__ == "__main__":
    stop_event = threading.Event()

    try:
        gps_thread = threading.Thread(target=run_gps, args=(stop_event,))
        imu_thread = threading.Thread(target=run_imu, args=(stop_event,))
        servo_thread = threading.Thread(target=run_servo, args=(stop_event,))

        gps_thread.start()
        imu_thread.start()
        servo_thread.start()

        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nZatrzymywanie programu...")
        stop_event.set()
        gps_thread.join()
        imu_thread.join()
        servo_thread.join()
        print("Zamknięto poprawnie.")
