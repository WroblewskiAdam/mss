import numpy as np
import time
from filterpy.kalman import KalmanFilter  # Poprawny import z FilterPy
# import pyserial  # do GPS
# import smbus2  # do akcelerometru
# import adafruit_adxl345  # alternatywnie do akcelerometru

# ---- Parametry Filtra Kalmana ----
dt = 0.1  # Interwał czasowy (10 Hz)
dim_x = 6  # Wymiar stanu [x, y, vx, vy, ax, ay]
dim_z = 6  # Wymiar pomiaru [gps_x, gps_y, gps_vx, gps_vy, accel_ax, accel_ay] (potencjalnie wszystkie mierzone)

# Macierz przejścia stanu F (6x6)
F = np.array([[1, 0, dt, 0, 0.5*dt**2, 0],
              [0, 1, 0, dt, 0,       0.5*dt**2],
              [0, 0, 1, 0, dt,       0],
              [0, 0, 0, 1, 0,       dt],
              [0, 0, 0, 0, 1,       0],
              [0, 0, 0, 0, 0,       1]])

# Macierz obserwacji H (6x6) - mierzymy wszystko
H = np.eye(dim_z)

# Macierz szumu procesu Q (dostosuj!)
q_pos = 0.01
q_vel = 0.01
q_accel = 0.1
Q = np.diag([q_pos, q_pos, q_vel, q_vel, q_accel, q_accel])

# Macierz szumu pomiarowego R (dla GPS i Akcelerometru - dostosuj!)
r_gps_pos = 1.0    # Wariancja pozycji GPS
r_gps_vel = 0.5    # Wariancja prędkości GPS
r_accel = 0.1      # Wariancja przyspieszenia akcelerometru
R = np.diag([r_gps_pos, r_gps_pos, r_gps_vel, r_gps_vel, r_accel, r_accel])


# ---- Inicjalizacja Filtra Kalmana z FilterPy ----
kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z) # Inicjalizacja wymiarów
kf.F = F
kf.H = H
kf.Q = Q
kf.R = R
kf.x = np.zeros(dim_x)       # Początkowy stan (wektor)
kf.P = np.eye(dim_x) * 100.0 # Początkowa kowariancja (macierz)


# ---- Inicjalizacja Czujników ----
# gps_serial = serial.Serial(...) # Inicjalizacja portu szeregowego GPS
# i2c_bus = smbus2.SMBus(1) # Inicjalizacja magistrali I2C
# accelerometer = adafruit_adxl345.ADXL345(i2c_bus) # Inicjalizacja akcelerometru

last_gps_time = time.time()
gps_interval = 1.0 # GPS 1Hz

def read_gps_data():
    # ... (kod do odczytu i parsowania danych GPS z pyserial) ...
    # ... (wyciągnij szerokość, długość geograficzną, prędkość, przekonwertuj na x, y, vx, vy w płaskim układzie) ...
    # ... (zwróć [pozycja_x, pozycja_y, prędkość_x, prędkość_y]) ...
    return [None, None, None, None] # Zwróć None, jeśli brak danych GPS

def read_accelerometer_data():
    # ... (kod do odczytu danych z akcelerometru z smbus2/adafruit_adxl345) ...
    # ... (przelicz na m/s²) ...
    # ... (zwróć [ax, ay, az]) - tutaj zakładamy, że interesują nas tylko ax i ay w płaszczyźnie XY) ...
    return [0.0, 0.0, 0.0] # Zwróć 0 na początek, albo aktualne odczyty


# ---- Pętla Główna ----
try:
    while True:
        start_time = time.time()

        # 1. Predykcja Filtra Kalmana (używamy predict z FilterPy)
        kf.predict()

        # 2. Odczyt Danych GPS (co sekundę)
        current_time = time.time()
        gps_x, gps_y, gps_vx, gps_vy = read_gps_data() # Odczyt GPS

        # 3. Odczyt Danych Akcelerometru (z częstotliwością 100 Hz)
        accel_ax, accel_ay, accel_az = read_accelerometer_data() # Odczyt akcelerometru

        # 4. Aktualizacja Filtra Kalmana (używamy update z FilterPy)
        if current_time - last_gps_time >= gps_interval and gps_x is not None and gps_y is not None and gps_vx is not None and gps_vy is not None:
            z_pomiar = np.array([gps_x, gps_y, gps_vx, gps_vy, accel_ax, accel_ay]) # Wektor pomiarów z GPS i Akcelerometru
            kf.update(z_pomiar) # Aktualizacja filtra z danymi GPS i Akcelerometru
            last_gps_time = current_time
        else:
            # Aktualizacja tylko danymi z akcelerometru - **ważna zmiana**:
            z_accel_only = np.array([None, None, None, None, accel_ax, accel_ay]) # Tylko pomiar z akcelerometru, None dla GPS
            z_pomiar_fillna = np.array([gps_x if gps_x is not None else kf.x[0], # Użyj ostatniej estymaty, jeśli brak GPS
                                        gps_y if gps_y is not None else kf.x[1],
                                        gps_vx if gps_vx is not None else kf.x[2],
                                        gps_vy if gps_vy is not None else kf.x[3],
                                        accel_ax, accel_ay]) # Wektor pomiarów z Akcelerometru i ewentualnie ostatniej estymaty GPS
            kf.update(z_pomiar_fillna, H=H, R=R) # Aktualizacja filtra z danymi Akcelerometru i ewentualnie ostatnią estymatą GPS, **jawne przekazanie H i R - potencjalnie niepotrzebne, ale dla pewności**


        # 5. Wyjście Wyników (pozycja i prędkość z filtra Kalmana)
        pozycja_x = kf.x[0]
        pozycja_y = kf.x[1]
        predkosc_x = kf.x[2]
        predkosc_y = kf.x[3]
        przyspieszenie_x = kf.x[4]
        przyspieszenie_y = kf.x[5]

        print(f"Pozycja: ({pozycja_x:.2f}, {pozycja_y:.2f}), Prędkość: ({predkosc_x:.2f}, {predkosc_y:.2f}), Przyspieszenie: ({przyspieszenie_x:.2f}, {przyspieszenie_y:.2f})")

        # 6. Kontrola Częstotliwości Wyjścia (10 Hz)
        elapsed_time = time.time() - start_time
        sleep_time = dt - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("Zakończono program")
finally:
    # ... (zamknij porty szeregowe, etc. - jeśli używane) ...
    pass