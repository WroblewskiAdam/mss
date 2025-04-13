import time
import board
import busio
import math
from datetime import datetime

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Inicjalizacja I2C i czujnika
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
time.sleep(1)

# Włączanie funkcji czujnika
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

# Konwersja kwaternionów do kątów
def quaternion_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# Parametry częstotliwości
interval = 0.1  # 100 ms = 10 Hz
last_time = time.perf_counter()
while True:
    now = time.perf_counter()
    if now - last_time >= interval:
        last_time = now

        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

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
