import time
import board
import busio
import imufusion
import numpy as np
from datetime import datetime


from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C


# Inicjalizacja I2C i BNO085
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
time.sleep(1)

# Włącz wymagane czujniki
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)

# Parametry próbkowania
sample_rate_hz = 100
interval = 1.0 / sample_rate_hz

# Inicjalizacja filtra i offsetu
offset = imufusion.Offset(sample_rate_hz)
ahrs = imufusion.Ahrs()

# Zaawansowane ustawienia filtra
ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_ENU,  # lub ENU, jeśli chcesz
    
    0.5,  # gain (czułość fuzji)
    2000,  # zakres żyroskopu [°/s]
    10,  # odrzucenie akcelerometru
    10,  # odrzucenie magnetometru
    5 * sample_rate_hz,  # trigger dla recovery
)


# Pętla pomiarowa
last_time = time.perf_counter()
while True:
    now = time.perf_counter()
    dt = now - last_time # Użyj rzeczywistego dt dla większej dokładności
    if dt >= interval: # Sprawdzamy, czy minął interwał, ale używamy dokładnego dt
        last_time = now

        # Zbierz dane
        accel_raw = bno.acceleration  # m/s^2
        gyro_raw = bno.gyro         # rad/s
        mag_raw = bno.magnetic      # µT

        # Sprawdzenie, czy dane są dostępne (BNO08x może nie zwracać danych w każdej chwili)
        if accel_raw is None or gyro_raw is None or mag_raw is None:
            # print("Dane z czujnika niekompletne, pomijam iterację.")
            time.sleep(interval / 10) # Krótka pauza, żeby nie zapętlić CPU na 100%
            continue

        # Jednostki:
        accel_g = np.array(accel_raw) / 9.80665  # m/s^2 → g
        gyro_deg = np.degrees(np.array(gyro_raw))  # rad/s → deg/s
        mag_uT = np.array(mag_raw)  # µT - już dobre

        # Korekcja offsetu żyroskopu
        gyro_deg_corrected = offset.update(gyro_deg)

        # Aktualizacja filtra Madgwicka
        ahrs.update(gyro_deg_corrected, accel_g, mag_uT, dt) # Przekaż rzeczywisty dt

        # --- Wyciąganie danych z algorytmu ---

        # 1. Kwaternion i kąty Eulera
        quaternion = ahrs.quaternion
        euler = quaternion.to_euler() # [Roll, Pitch, Yaw] w stopniach
        # Opcjonalnie: macierz rotacji
        # rotation_matrix = quaternion.to_matrix()

        # 2. Wektor grawitacji (w układzie czujnika)
        gravity_sensor_frame = ahrs.gravity # numpy array [gx, gy, gz] w 'g'

        # 3. Przyspieszenie liniowe (w układzie czujnika, bez grawitacji)
        linear_accel_sensor_frame = ahrs.linear_acceleration # numpy array [ax, ay, az] w 'g'

        # 4. Przyspieszenie ziemskie (w układzie Ziemi NED, bez grawitacji)
        # Dla NED: X -> Północ, Y -> Wschód, Z -> Dół
        earth_accel_earth_frame = ahrs.earth_acceleration # numpy array [a_N, a_E, a_D] w 'g'

        # --- Wyświetlanie danych ---
        print(f"Euler [deg]: Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")
        # print(f"Quaternion: w={quaternion.w:.3f}, x={quaternion.x:.3f}, y={quaternion.y:.3f}, z={quaternion.z:.3f}")
        # if 'rotation_matrix' in locals():
        #     print(f"Rotation Matrix:\n{rotation_matrix}")

        print(f"Gravity (sensor) [g]: X={gravity_sensor_frame[0]:.2f}, Y={gravity_sensor_frame[1]:.2f}, Z={gravity_sensor_frame[2]:.2f}")
        print(f"Linear Accel (sensor) [g]: X={linear_accel_sensor_frame[0]:.2f}, Y={linear_accel_sensor_frame[1]:.2f}, Z={linear_accel_sensor_frame[2]:.2f}")
        
        # Nazewnictwo osi dla earth_accel_earth_frame zależy od ustawionej konwencji
        # Dla CONVENTION_NED (domyślne ustawienie w Twoim kodzie, jeśli 0.5 to gain):
        # X to Północ (North), Y to Wschód (East), Z to Dół (Down)
        print(f"Earth Accel (NED) [g]: North={earth_accel_earth_frame[0]:.2f}, East={earth_accel_earth_frame[1]:.2f}, Down={earth_accel_earth_frame[2]:.2f}")
        print("-" * 30)

    else:
        # Jeśli nie minął interwał, można dać krótki sleep, aby nie obciążać CPU
        # To zależy od tego, jak szybko chcesz reagować na nowe dane i jak dokładny ma być 'interval'
        time.sleep(max(0, interval - dt) * 0.9) # Śpij przez pozostały czas interwału (z małym marginesem)