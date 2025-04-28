import numpy as np
import pandas as pd
from filterpy.kalman import ExtendedKalmanFilter
# from filterpy.common import Q_discrete_white_noise # Nie używamy tej funkcji bezpośrednio
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
# from datetime import datetime, timedelta # Nie są już potrzebne bezpośrednio
# import math # Nie jest już potrzebny bezpośrednio
from collections import deque # Potrzebne dla bufora ZUPT

# --- Funkcje pomocnicze (bez zmian) ---

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def quaternion_conjugate(q):
    q_conj = np.copy(q)
    q_conj[1:] *= -1
    return q_conj

def rotate_vector_by_quaternion(v, q):
    q_v = np.concatenate(([0], v))
    q_rotated = quaternion_multiply(quaternion_multiply(q, q_v), quaternion_conjugate(q))
    return q_rotated[1:]

def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm < 1e-9: # Unikaj dzielenia przez zero
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm

def ned_to_lla(ned, ref_lla):
    lat0, lon0, alt0 = np.radians(ref_lla[0]), np.radians(ref_lla[1]), ref_lla[2]
    R_earth = 6378137.0
    d_north, d_east, d_down = ned
    dlat = np.degrees(d_north / R_earth)
    # Zabezpieczenie przed dzieleniem przez zero na biegunach (mało prawdopodobne, ale bezpieczne)
    cos_lat0 = np.cos(lat0)
    if abs(cos_lat0) < 1e-9:
        cos_lat0 = 1e-9
    dlon = np.degrees(d_east / (R_earth * cos_lat0))
    new_lat = ref_lla[0] + dlat
    new_lon = ref_lla[1] + dlon
    new_alt = alt0 - d_down
    return np.array([new_lat, new_lon, new_alt])

# --- Funkcja detekcji postoju (ZUPT) ---
ZUPT_GPS_SPEED_THRESHOLD = 0.15 # Próg prędkości GPS [m/s] - DOSTROJ!
ZUPT_ACCEL_STD_THRESHOLD = 0.2  # Próg odch. std. akcelerometru [m/s^2] - DOSTROJ!
ZUPT_GYRO_STD_THRESHOLD  = 0.03 # Próg odch. std. żyroskopu [rad/s] - DOSTROJ!
ZUPT_WINDOW_SIZE       = 25    # Rozmiar okna IMU do analizy (liczba próbek) - DOSTROJ!

def detect_standstill(imu_window, latest_gps_speed):
    """Wykrywa postój na podstawie danych z okna IMU i ostatniej prędkości GPS."""
    # 1. Sprawdź prędkość GPS
    if latest_gps_speed > ZUPT_GPS_SPEED_THRESHOLD:
        return False

    # 2. Sprawdź, czy mamy wystarczająco danych w oknie
    if len(imu_window) < ZUPT_WINDOW_SIZE:
        return False

    # 3. Wyodrębnij dane z bufora (deque) i UPEWNIJ SIĘ, że są typu float
    try:
        # Konwersja na float64 i obsługa potencjalnych błędów (np. jeśli 'reading' nie jest poprawnym arrayem)
        accel_data = np.array([reading[0:3] for reading in imu_window], dtype=np.float64)
        gyro_data = np.array([reading[3:6] for reading in imu_window], dtype=np.float64)
    except (ValueError, IndexError, TypeError) as e:
        # Jeśli wystąpi błąd podczas konwersji/wyodrębniania, nie możemy wykryć postoju
        print(f"Ostrzeżenie: Błąd podczas przygotowywania danych w detect_standstill: {e}")
        # Można dodać więcej informacji diagnostycznych, np. zawartość imu_window
        # print("Problematic imu_window content (first 5):", list(imu_window)[:5])
        return False

    # 4. Sprawdź, czy nie ma NaN lub Inf (dodatkowe zabezpieczenie)
    if np.isnan(accel_data).any() or np.isinf(accel_data).any() or \
       np.isnan(gyro_data).any() or np.isinf(gyro_data).any():
        print("Ostrzeżenie: Wykryto NaN lub Inf w danych okna IMU w detect_standstill.")
        return False

    # 5. Sprawdź odchylenie standardowe akcelerometru
    try:
        accel_std = np.std(accel_data, axis=0)
    except Exception as e:
        print(f"Błąd podczas obliczania np.std dla accel_data: {e}")
        return False # Nie można kontynuować

    if np.any(accel_std > ZUPT_ACCEL_STD_THRESHOLD):
        # print(f"Debug ZUPT: Accel std {accel_std} > {ZUPT_ACCEL_STD_THRESHOLD}")
        return False

    # 6. Sprawdź odchylenie standardowe żyroskopu
    try:
        gyro_std = np.std(gyro_data, axis=0)
    except Exception as e:
        print(f"Błąd podczas obliczania np.std dla gyro_data: {e}")
        return False # Nie można kontynuować

    if np.any(gyro_std > ZUPT_GYRO_STD_THRESHOLD):
        # print(f"Debug ZUPT: Gyro std {gyro_std} > {ZUPT_GYRO_STD_THRESHOLD}")
        return False

    # Jeśli wszystkie warunki spełnione
    # print(f"Debug ZUPT: Standstill detected! GPS Speed: {latest_gps_speed:.2f}, Accel std: {np.round(accel_std,3)}, Gyro std: {np.round(gyro_std,3)}")
    return True

# --- Definicja EKF (bez zmian w logice wewnętrznej) ---
class InsEkf(ExtendedKalmanFilter):
    def __init__(self, dt_imu, initial_state, initial_cov, Q_std_devs, R_gps_pos, R_gps_vel, R_mag):
        dim_x = 16
        dim_z_gps = 6
        super().__init__(dim_x=dim_x, dim_z=dim_z_gps)

        self.x = initial_state.copy() # Użyj kopii!
        self.P = initial_cov.copy() # Użyj kopii!
        self.dt_imu = dt_imu

        acc_std, gyro_std, acc_bias_std, gyro_bias_std = Q_std_devs
        self.Q = np.zeros((dim_x, dim_x))
        self.Q[3:6, 3:6] = np.diag([acc_std**2] * 3) * self.dt_imu
        gyro_noise_effect = (gyro_std * self.dt_imu)**2
        # Uproszczone przypisanie szumu do kwaternionu (lepsze metody istnieją)
        self.Q[6:10, 6:10] = np.diag([gyro_noise_effect]*4) * 0.25
        self.Q[10:13, 10:13] = np.diag([acc_bias_std**2] * 3) * self.dt_imu
        self.Q[13:16, 13:16] = np.diag([gyro_bias_std**2] * 3) * self.dt_imu
        self.Q[0:3, 0:3] = np.diag([1e-5**2]*3) * self.dt_imu # Szum pozycji (mały)

        self.R_gps_pos = np.diag(np.array(R_gps_pos)**2) # Upewnij się, że to kwadraty std dev
        self.R_gps_vel = np.diag(np.array(R_gps_vel)**2) # Upewnij się, że to kwadraty std dev
        self.R_gps_combined = np.block([
            [self.R_gps_pos, np.zeros((3,3))],
            [np.zeros((3,3)), self.R_gps_vel]
        ])
        self.R_mag = np.diag(np.array(R_mag)**2) # Upewnij się, że to kwadraty std dev

        self.g_ned = np.array([0, 0, 9.81])
        mag_north, mag_east, mag_down = 20.0, 1.0, 45.0 # Przykładowe wartości
        self.mag_ref_ned = np.array([mag_north, mag_east, mag_down])


    def predict(self, dt, u):
        if dt <= 0: # Dodatkowe zabezpieczenie
            return

        accel = u[:3]
        gyro = u[3:]

        pos = self.x[0:3]
        vel = self.x[3:6]
        q = normalize_quaternion(self.x[6:10])
        b_a = self.x[10:13]
        b_g = self.x[13:16]

        accel_corr = accel - b_a
        gyro_corr = gyro - b_g

        omega = gyro_corr
        dq_dt_w = -0.5 * (omega[0] * q[1] + omega[1] * q[2] + omega[2] * q[3])
        dq_dt_x =  0.5 * (omega[0] * q[0] + omega[1] * q[3] - omega[2] * q[2])
        dq_dt_y =  0.5 * (-omega[0] * q[3] + omega[1] * q[0] + omega[2] * q[1])
        dq_dt_z =  0.5 * (omega[0] * q[2] - omega[1] * q[1] + omega[2] * q[0])
        dq_dt = np.array([dq_dt_w, dq_dt_x, dq_dt_y, dq_dt_z])

        q_new = q + dq_dt * dt
        q_new = normalize_quaternion(q_new)

        # Użycie Rot.from_quat wymaga formatu [x, y, z, w]
        # Nasz kwaternion to [w, x, y, z]
        # Można użyć naszej funkcji rotate_vector... lub przekonwertować
        # accel_ned = rotate_vector_by_quaternion(accel_corr, q)
        # Alternatywnie z scipy:
        try:
            Rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix() # [x, y, z, w] format
            accel_ned = Rot_matrix @ accel_corr # Mnożenie macierzowe
        except ValueError: # Może się zdarzyć, jeśli kwaternion jest zły
             print("Ostrzeżenie: Błąd w konwersji kwaternionu w predykcji.")
             accel_ned = accel_corr # Użyj nieskorygowanej jako fallback

        vel_new = vel + (accel_ned - self.g_ned) * dt
        pos_new = pos + vel * dt + 0.5 * (accel_ned - self.g_ned) * dt**2 # Lepsza integracja pozycji

        b_a_new = b_a
        b_g_new = b_g

        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:10] = q_new
        self.x[10:13] = b_a_new
        self.x[13:16] = b_g_new

        # Jakobian F (nadal uproszczony)
        F = np.eye(self.dim_x)
        F[0:3, 3:6] = np.eye(3) * dt
        # dVel/dOrientacja - pominięte
        # dVel/dBiasAcc
        # Użyj tej samej macierzy rotacji co wyżej
        F[3:6, 10:13] = -Rot_matrix * dt
        # dOrientacja/dOrientacja - pominięte
        # dOrientacja/dBiasGyro - pominięte

        # Predykcja kowariancji
        # Stabilniejsza numerycznie forma: P = FPF' + Q
        self.P = F @ self.P @ F.T + self.Q

    def H_gps_jacobian(self, x):
        H = np.zeros((6, self.dim_x))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        return H

    def h_gps(self, x):
        return np.concatenate((x[0:3], x[3:6]))

    # --- Funkcje dla ZUPT ---
    def H_zupt_jacobian(self, x):
        """Jakobian dla pomiaru zerowej prędkości."""
        H = np.zeros((3, self.dim_x))
        H[:, 3:6] = np.eye(3) # Pochodna prędkości (x[3:6]) po prędkości (x[3:6])
        return H

    def h_zupt(self, x):
        """Funkcja pomiaru dla zerowej prędkości."""
        return x[3:6] # Mierzymy bezpośrednio estymowaną prędkość

    # --- Funkcje Mag (bez zmian) ---
    def H_mag_jacobian(self, x):
        H = np.zeros((3, self.dim_x))
        # Implementacja wymaga pochodnych h_mag po kwaternionie
        return H

    def h_mag(self, x):
        q = normalize_quaternion(x[6:10])
        q_conj = quaternion_conjugate(q)
        mag_body_predicted = rotate_vector_by_quaternion(self.mag_ref_ned, q_conj)
        return mag_body_predicted

# --- Ładowanie i przygotowanie danych (bez zmian) ---
print("Ładowanie danych...")
# Użyj poprawnych ścieżek do plików
gps_file_path = 'D:/Studia/MGR/Magisterka/data/27-04-25/pi4/gps_data_tractor_v3.csv'
imu_file_path = 'D:/Studia/MGR/Magisterka/data/27-04-25/pi4/imu_data_tractor_v3.csv'

try:
    gps_data_org = pd.read_csv(gps_file_path)
    imu_data = pd.read_csv(imu_file_path)
    print("Dane załadowane.")
except FileNotFoundError:
    print(f"Błąd: Nie znaleziono plików danych w '{gps_file_path}' lub '{imu_file_path}'.")
    exit()

print("Kolumny GPS:", gps_data_org.columns)
print("Kolumny IMU:", imu_data.columns)

try:
    gps_data_org.columns = gps_data_org.columns.str.strip()
    imu_data.columns = imu_data.columns.str.strip()

    gps_time_col = 'Local_Time' if 'Local_Time' in gps_data_org.columns else 'Timestamp'
    imu_time_col = 'Timestamp'

    # Spróbuj różnych formatów czasu
    try: gps_data_org['TimestampEpoch'] = pd.to_datetime(gps_data_org[gps_time_col], format='%H:%M:%S.%f').apply(lambda t: t.timestamp())
    except ValueError:
        try: gps_data_org['TimestampEpoch'] = pd.to_datetime(gps_data_org[gps_time_col], format='%H:%M:%S').apply(lambda t: t.timestamp())
        except ValueError: gps_data_org['TimestampEpoch'] = pd.to_numeric(gps_data_org[gps_time_col])

    try: imu_data['TimestampEpoch'] = pd.to_datetime(imu_data[imu_time_col], format='%Y-%m-%d %H:%M:%S.%f').apply(lambda t: t.timestamp())
    except ValueError:
         try: imu_data['TimestampEpoch'] = pd.to_datetime(imu_data[imu_time_col], format='%H:%M:%S.%f').apply(lambda t: t.timestamp())
         except ValueError: imu_data['TimestampEpoch'] = pd.to_numeric(imu_data[imu_time_col])

    gps_data = gps_data_org.iloc[14:].copy()
    # Sprawdź, czy są dane po filtrowaniu
    if gps_data.empty:
        print("Błąd: Brak danych GPS po odfiltrowaniu początkowych wierszy.")
        exit()
    start_time_epoch = gps_data['TimestampEpoch'].iloc[0]

    gps_data['RelTime'] = gps_data['TimestampEpoch'] - start_time_epoch
    imu_data['RelTime'] = imu_data['TimestampEpoch'] - start_time_epoch

    if 'Speed' in gps_data.columns and 'Heading' in gps_data.columns:
       gps_data['Speed_mps'] = gps_data['Speed'] / 3.6
       heading_rad = np.radians(gps_data['Heading'])
       gps_data['VelN'] = gps_data['Speed_mps'] * np.cos(heading_rad)
       gps_data['VelE'] = gps_data['Speed_mps'] * np.sin(heading_rad)
       gps_data['VelD'] = 0.0
    else:
       print("Ostrzeżenie: Brak kolumn 'Speed' lub 'Heading'. Wypełniam prędkość GPS zerami.")
       gps_data['Speed_mps'] = 0.0
       gps_data['VelN'] = 0.0
       gps_data['VelE'] = 0.0
       gps_data['VelD'] = 0.0

    accel_cols = ['LinAccel_X', 'LinAccel_Y', 'LinAccel_Z']
    gyro_cols = ['Gyro_X', 'Gyro_Y', 'Gyro_Z']
    mag_cols = ['Mag_X', 'Mag_Y', 'Mag_Z']

    if not all(col in imu_data.columns for col in accel_cols + gyro_cols): # Mag nie jest krytyczny
        print("Błąd: Brak wymaganych kolumn IMU (LinAccel_*, Gyro_*)")
        exit()
    if not all(col in imu_data.columns for col in mag_cols):
        print("Ostrzeżenie: Brak kolumn Mag_*. Fuzja magnetometru nie będzie możliwa.")
        # Wypełnij zerami, jeśli brak, aby kod działał dalej
        for col in mag_cols:
             if col not in imu_data.columns: imu_data[col] = 0.0


    accel = imu_data[accel_cols].values
    gyro = imu_data[gyro_cols].values
    mag = imu_data[mag_cols].values

    # Znajdź pierwszy indeks IMU po pierwszym czasie GPS
    first_gps_time = gps_data['RelTime'].iloc[0]
    valid_imu_indices = imu_data.index[imu_data['RelTime'] >= first_gps_time]
    if not valid_imu_indices.any():
         print("Błąd: Brak danych IMU po pierwszym czasie GPS.")
         exit()
    first_good_imu_idx = valid_imu_indices[0]

    # Pobierz indeks w oryginalnej ramce danych przed resetowaniem indeksu
    original_index_val = imu_data.loc[first_good_imu_idx].name

    imu_data = imu_data.loc[original_index_val:].reset_index(drop=True)

    # Dopasuj dane numpy do nowej ramki danych
    accel = imu_data[accel_cols].values
    gyro = imu_data[gyro_cols].values
    mag = imu_data[mag_cols].values


    imu_sec = imu_data['RelTime'].values
    gps_sec = gps_data['RelTime'].values

    print(f"Pierwszy znacznik czasu GPS (względny): {gps_sec[0]:.3f} s")
    print(f"Pierwszy znacznik czasu IMU po sync (względny): {imu_sec[0]:.3f} s")

except KeyError as e:
    print(f"Błąd: Brak kluczowej kolumny w danych: {e}")
    exit()
except IndexError as e:
     print(f"Błąd indeksowania danych: {e}. Może być problem z synchronizacją czasu lub puste ramki danych.")
     exit()
except Exception as e:
    print(f"Wystąpił nieoczekiwany błąd podczas przetwarzania danych: {e}")
    exit()


# --- Połącz i posortuj dane (bez zmian) ---
imu_data['Type'] = 'IMU'
gps_data['Type'] = 'GPS'
imu_subset = imu_data[['RelTime', 'Type', 'LinAccel_X', 'LinAccel_Y', 'LinAccel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Mag_X', 'Mag_Y', 'Mag_Z']].copy()
imu_subset.rename(columns={'RelTime': 'Time'}, inplace=True)
gps_subset = gps_data[['RelTime', 'Type', 'Latitude', 'Longitude', 'Altitude', 'VelN', 'VelE', 'VelD', 'Speed_mps']].copy() # Dodaj Speed_mps
gps_subset.rename(columns={'RelTime': 'Time', 'Altitude': 'Alt'}, inplace=True)

all_data = pd.concat([imu_subset, gps_subset], ignore_index=True)
all_data.sort_values(by='Time', inplace=True)
all_data.reset_index(drop=True, inplace=True)

print("Połączone i posortowane dane (pierwsze 5 wierszy):")
print(all_data.head())


# --- Inicjalizacja Filtru (bez zmian) ---
print("Inicjalizacja EKF...")
imu_fs = 100
dt_imu = 1.0 / imu_fs
ref_lla = [gps_data['Latitude'].iloc[0], gps_data['Longitude'].iloc[0], gps_data['Alt'].iloc[0] if 'Alt' in gps_data.columns and not pd.isna(gps_data['Alt'].iloc[0]) else 0]

initial_state = np.zeros(16)
initial_state[3:6] = [gps_data['VelN'].iloc[0], gps_data['VelE'].iloc[0], gps_data['VelD'].iloc[0]]
initial_state[6] = 1.0

initial_cov = np.diag([
    0.1**2, 0.1**2, 0.1**2,    # Pos P (m^2) - Mniejsza niepewność początkowa
    0.1**2, 0.1**2, 0.1**2,    # Vel P ((m/s)^2) - Mniejsza niepewność
    np.radians(5)**2, np.radians(5)**2, np.radians(5)**2, np.radians(5)**2, # Orient P (rad^2) ~5 deg
    (0.05)**2, (0.05)**2, (0.05)**2, # Acc Bias P ((m/s^2)^2)
    (np.radians(0.1))**2, (np.radians(0.1))**2, (np.radians(0.1))**2 # Gyro Bias P ((rad/s)^2)
])

# === NOWE WARTOŚCI Q i R === (Bazując na poprzednich sugestiach, ale nadal wymagają strojenia)
Q_std_devs = [
    1.0,     # Accel Noise (m/s^2 / sqrt(Hz)) - Zwiększone!
    2e-2,    # Gyro Noise (rad/s / sqrt(Hz)) - Lekko zwiększone
    5e-3,    # Accel Bias Random Walk (m/s^3 / sqrt(Hz)) - Zwiększone!
    2e-4     # Gyro Bias Random Walk (rad/s^2 / sqrt(Hz)) - Zwiększone!
]

RTK_FIXED_POS_STD_HORIZ = 0.04 # [m]
RTK_FIXED_POS_STD_VERT = 0.03  # [m] - Zwykle większa niepewność pionowa
RTK_FIXED_VEL_STD      = 0.03  # [m/s] - Trochę mniej optymistycznie
MAG_STD = 5.0                  # [uT lub inna jednostka] - Większa niepewność Mag

R_gps_pos = [RTK_FIXED_POS_STD_HORIZ, RTK_FIXED_POS_STD_HORIZ, RTK_FIXED_POS_STD_VERT]
R_gps_vel = [RTK_FIXED_VEL_STD, RTK_FIXED_VEL_STD, RTK_FIXED_VEL_STD]
R_mag = [MAG_STD, MAG_STD, MAG_STD]

ekf = InsEkf(dt_imu, initial_state, initial_cov, Q_std_devs, R_gps_pos, R_gps_vel, R_mag)

# --- Macierz R dla ZUPT ---
# Jak bardzo ufamy, że prędkość jest zerowa, gdy ZUPT jest aktywny?
# Mniejsza wartość = większe zaufanie
ZUPT_VEL_STD = 0.01 # [m/s] - DOSTROJ!
R_zupt = np.diag([ZUPT_VEL_STD**2] * 3)


# --- Główna pętla filtru ---
print("Rozpoczynanie filtrowania...")
num_samples = len(all_data)
est_pos_ned = np.zeros((num_samples, 3))
est_vel_ned = np.zeros((num_samples, 3))
est_orient_quat = np.zeros((num_samples, 4))
est_acc_bias = np.zeros((num_samples, 3))
est_gyro_bias = np.zeros((num_samples, 3))
timestamps = np.zeros(num_samples)
zupt_active_flags = np.zeros(num_samples, dtype=bool) # Flaga do śledzenia ZUPT

# Inicjalizacja bufora IMU dla ZUPT
imu_window = deque(maxlen=ZUPT_WINDOW_SIZE)
# Inicjalizacja ostatniej prędkości GPS
latest_gps_speed = gps_data['Speed_mps'].iloc[0] if not gps_data.empty else 0.0

last_time = all_data['Time'].iloc[0] if num_samples > 0 else 0.0

# Zmienne do przechowywania ostatniego odczytu IMU dla predykcji
last_imu_reading = np.zeros(6) # [ax, ay, az, gx, gy, gz]
# Sprawdź, czy pierwszy wpis to IMU, aby zainicjować last_imu_reading
if num_samples > 0 and all_data['Type'].iloc[0] == 'IMU':
     first_row = all_data.iloc[0]
     last_imu_reading = np.concatenate((
          first_row[['LinAccel_X', 'LinAccel_Y', 'LinAccel_Z']].values,
          first_row[['Gyro_X', 'Gyro_Y', 'Gyro_Z']].values
     ))


for i in range(num_samples):
    row = all_data.iloc[i]
    current_time = row['Time']
    data_type = row['Type']

    # --- Krok predykcji ---
    dt = current_time - last_time
    # Unikaj ujemnych lub zerowych dt
    if dt < 1e-9:
      # Jeśli dt jest bardzo małe, ale to ten sam typ danych, po prostu zaktualizuj czas
      if i > 0 and data_type == all_data['Type'].iloc[i-1]:
          last_time = current_time
          # Zaktualizuj dane wyjściowe dla tego samego czasu, jeśli trzeba
          timestamps[i] = current_time
          est_pos_ned[i, :] = ekf.x[0:3]
          est_vel_ned[i, :] = ekf.x[3:6]
          est_orient_quat[i, :] = ekf.x[6:10]
          est_acc_bias[i, :] = ekf.x[10:13]
          est_gyro_bias[i, :] = ekf.x[13:16]
          zupt_active_flags[i] = zupt_active_flags[i-1] if i > 0 else False # Kopiuj flagę ZUPT
          continue # Przejdź do następnej iteracji
      else:
          dt = 1e-9 # Użyj minimalnego kroku, jeśli czas się nie zmienił, ale typ danych tak

    if dt > 0.5:
        print(f"Ostrzeżenie: Duży krok czasowy dt={dt:.3f}s przy indeksie {i} ({data_type}).")

    # Wykonaj predykcję tylko jeśli dt > 0
    if dt > 1e-9:
        if np.any(last_imu_reading):
            ekf.predict(dt=dt, u=last_imu_reading)
        else:
            # Jeśli nie ma odczytu IMU, tylko propaguj kowariancję (bardzo uproszczone)
             F = np.eye(ekf.dim_x) # Zakładamy brak zmiany stanu
             ekf.P = F @ ekf.P @ F.T + ekf.Q
             # print(f"Ostrzeżenie: Brak danych IMU do predykcji przy i={i}, dt={dt}")


    # --- Zapisz estymowany stan (po predykcji, przed aktualizacją) ---
    timestamps[i] = current_time
    est_pos_ned[i, :] = ekf.x[0:3]
    est_vel_ned[i, :] = ekf.x[3:6]
    est_orient_quat[i, :] = ekf.x[6:10]
    est_acc_bias[i, :] = ekf.x[10:13]
    est_gyro_bias[i, :] = ekf.x[13:16]
    # Domyślnie flaga ZUPT jest False, zostanie ustawiona na True jeśli ZUPT zostanie zastosowany
    zupt_active_flags[i] = False

    # --- Krok aktualizacji ---
    is_stationary = False # Resetuj flagę na początku każdej iteracji
    if data_type == 'IMU':
        accel = row[['LinAccel_X', 'LinAccel_Y', 'LinAccel_Z']].values
        gyro = row[['Gyro_X', 'Gyro_Y', 'Gyro_Z']].values
        mag = row[['Mag_X', 'Mag_Y', 'Mag_Z']].values if all(c in row for c in mag_cols) else np.zeros(3)
        last_imu_reading = np.concatenate((accel, gyro)) # Zaktualizuj ostatni odczyt

        # Dodaj bieżący odczyt IMU do okna ZUPT
        imu_window.append(last_imu_reading)

        # --- Sprawdź warunki ZUPT ---
        # is_stationary = detect_standstill(imu_window, latest_gps_speed)
        is_stationary = False
        if is_stationary:
            zupt_active_flags[i] = True # Ustaw flagę
            z_zupt = np.zeros(3) # Pomiar zerowej prędkości
            try:
                # Użyj dedykowanych funkcji/lambda dla H i h
                ekf.update(z_zupt, R=R_zupt, HJacobian=ekf.H_zupt_jacobian, Hx=ekf.h_zupt)
                # print(f"Applied ZUPT at {current_time:.3f}")
            except Exception as e:
                print(f"Błąd podczas ZUPT (indeks {i}): {e}")
                # print("Stan x:", ekf.x)
                # print("Kowariancja P diag:", np.diag(ekf.P))

        # Opcjonalna fuzja Mag (jeśli H_mag zaimplementowane)
        # if not is_stationary and 'Mag_X' in row and np.linalg.norm(mag) > 1e-6 : # Fuzuj Mag tylko w ruchu?
        #     try:
        #         # ekf.update(mag, R=ekf.R_mag, HJacobian=ekf.H_mag_jacobian, Hx=ekf.h_mag)
        #         pass # Odkomentuj, jeśli H_mag jest gotowe
        #     except Exception as e:
        #         print(f"Błąd podczas fuzji magnetometru: {e}")


    elif data_type == 'GPS':
        lat, lon = row['Latitude'], row['Longitude']
        alt = row['Alt'] if 'Alt' in row and not pd.isna(row['Alt']) else ref_lla[2]

        R_earth = 6378137.0
        pos_ned_meas = np.array([
            np.radians(lat - ref_lla[0]) * R_earth,
            np.radians(lon - ref_lla[1]) * R_earth * np.cos(np.radians(ref_lla[0])),
            -(alt - ref_lla[2])
        ])
        vel_ned_meas = row[['VelN', 'VelE', 'VelD']].values

        # Zaktualizuj ostatnią prędkość GPS (używamy normy horyzontalnej)
        latest_gps_speed = row['Speed_mps'] # Użyj bezpośrednio Speed_mps

        z_gps = np.concatenate((pos_ned_meas, vel_ned_meas))

        # Wykonaj aktualizację GPS tylko jeśli pojazd nie jest wykryty jako stacjonarny przez ZUPT
        # LUB jeśli ZUPT nie jest aktywny (dla pewności)
        # Można też fuzjować GPS zawsze, ale ZUPT zwykle dominuje przy postoju.
        # if not is_stationary: # Fuzuj GPS tylko w ruchu
        try:
            ekf.update(z_gps, R=ekf.R_gps_combined, HJacobian=ekf.H_gps_jacobian, Hx=ekf.h_gps)
            # print(f"Fused GPS at {current_time:.3f} (Index {i})")
        except Exception as e:
             print(f"Błąd podczas fuzji GPS (indeks {i}): {e}")
             # print("Stan x:", ekf.x)
             # print("Kowariancja P diag:", np.diag(ekf.P))


    # Zaktualizuj czas na koniec pętli
    last_time = current_time

print("Filtrowanie zakończone.")

# --- Wyniki i Wizualizacja ---
print("Przygotowanie wyników...")

# Usunięcie duplikatów czasowych (jeśli istnieją) - Ważne!
unique_indices = np.unique(timestamps, return_index=True)[1]
if len(unique_indices) < len(timestamps):
     print(f"Usunięto {len(timestamps) - len(unique_indices)} duplikatów czasowych przy wizualizacji.")
     timestamps = timestamps[unique_indices]
     est_pos_ned = est_pos_ned[unique_indices]
     est_vel_ned = est_vel_ned[unique_indices]
     est_orient_quat = est_orient_quat[unique_indices]
     est_acc_bias = est_acc_bias[unique_indices]
     est_gyro_bias = est_gyro_bias[unique_indices]
     zupt_active_flags = zupt_active_flags[unique_indices]


est_speed = np.linalg.norm(est_vel_ned[:, 0:2], axis=1)
# Użyj oryginalnych danych GPS do porównania prędkości
gps_speed = gps_data['Speed_mps'].values
gps_time_for_plot = gps_data['RelTime'].values # Użyj RelTime z gps_data

est_pos_lla = np.array([ned_to_lla(ned, ref_lla) for ned in est_pos_ned])

print("Rysowanie wyników...")

# 1. Trajektoria (bez zmian)
plt.figure(figsize=(10, 8))
plt.plot(gps_data['Longitude'], gps_data['Latitude'], 'r.-', label='GPS Raw', markersize=4, alpha=0.7)
plt.plot(est_pos_lla[:, 1], est_pos_lla[:, 0], 'b-', label='EKF Estimate', linewidth=1.5)
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Trajektoria GPS vs EKF")
plt.legend()
plt.grid(True)
plt.axis('equal')

# 2. Prędkość z zaznaczeniem ZUPT
plt.figure(figsize=(14, 7))
plt.plot(timestamps, est_speed, 'b-', label='EKF Speed', linewidth=1)
plt.plot(gps_time_for_plot, gps_speed, 'ro', label='GPS Speed', markersize=4, alpha=0.8)

# Wygładzona prędkość (opcjonalnie)
window_len_sec = 1.0 # Krótsze okno wygładzania?
window_len_samples = int(window_len_sec * imu_fs)
if len(est_speed) > window_len_samples:
    # Użyj trybu 'same' dla łatwiejszego dopasowania czasu
    est_speed_smoothed = np.convolve(est_speed, np.ones(window_len_samples)/window_len_samples, mode='same')
    plt.plot(timestamps, est_speed_smoothed, 'g-', label=f'EKF Smoothed ({window_len_sec}s window)', linewidth=1.5)

# Zaznacz obszary, gdzie ZUPT był aktywny
zupt_indices = np.where(zupt_active_flags)[0]
if len(zupt_indices) > 0:
    # Stwórz bloki cieniowania dla aktywności ZUPT
    # To może być bardziej skomplikowane, jeśli ZUPT włącza się i wyłącza często
    # Prostsze podejście: narysuj pionowe linie lub punkty
    plt.plot(timestamps[zupt_indices], np.zeros_like(timestamps[zupt_indices]) - 0.1, 'm|', markersize=10, label='ZUPT Active')
    # Można też użyć fill_between, ale wymaga to znalezienia ciągłych bloków ZUPT
    # plt.fill_between(timestamps, -0.2, -0.1, where=zupt_active_flags, color='magenta', alpha=0.3, label='ZUPT Active', step='mid')


plt.xlabel("Czas (s)")
plt.ylabel("Prędkość (m/s)")
plt.title("Estymowana prędkość vs Prędkość GPS (z ZUPT)")
plt.legend()
plt.grid(True)
plt.ylim(bottom=-0.2) # Zostaw trochę miejsca na wskaźnik ZUPT
plt.ylim(top=max(np.max(est_speed)*1.1, np.max(gps_speed)*1.1)) # Dopasuj górną granicę

# 3. Komponenty prędkości (bez zmian)
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(timestamps, est_vel_ned[:, 0], 'b-', label='EKF Vel North')
plt.plot(gps_time_for_plot, gps_data['VelN'], 'ro', label='GPS Vel North', alpha=0.7)
plt.legend()
plt.grid(True)
plt.ylabel("Vel N (m/s)")
plt.title("Komponenty prędkości NED")

plt.subplot(3, 1, 2)
plt.plot(timestamps, est_vel_ned[:, 1], 'b-', label='EKF Vel East')
plt.plot(gps_time_for_plot, gps_data['VelE'], 'ro', label='GPS Vel East', alpha=0.7)
plt.legend()
plt.grid(True)
plt.ylabel("Vel E (m/s)")

plt.subplot(3, 1, 3)
plt.plot(timestamps, est_vel_ned[:, 2], 'b-', label='EKF Vel Down')
# plt.plot(gps_time_for_plot, gps_data['VelD'], 'ro', label='GPS Vel Down', alpha=0.7) # VelD GPS często szumne
plt.legend()
plt.grid(True)
plt.ylabel("Vel D (m/s)")
plt.xlabel("Czas (s)")
plt.tight_layout()

# 4. Estymowane Biassy (Dryfty) - bardzo przydatne do diagnostyki
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(timestamps, est_acc_bias[:, 0], label='Bias Acc X')
plt.plot(timestamps, est_acc_bias[:, 1], label='Bias Acc Y')
plt.plot(timestamps, est_acc_bias[:, 2], label='Bias Acc Z')
plt.title('Estymowane Dryfty Akcelerometru')
plt.ylabel('Dryft (m/s^2)')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(timestamps, np.degrees(est_gyro_bias[:, 0]), label='Bias Gyro X') # Konwersja na deg/s dla czytelności
plt.plot(timestamps, np.degrees(est_gyro_bias[:, 1]), label='Bias Gyro Y')
plt.plot(timestamps, np.degrees(est_gyro_bias[:, 2]), label='Bias Gyro Z')
plt.title('Estymowane Dryfty Żyroskopu')
plt.ylabel('Dryft (deg/s)')
plt.grid(True)
plt.legend()
plt.xlabel('Czas (s)')

plt.tight_layout()
plt.show()

print("Gotowe.")