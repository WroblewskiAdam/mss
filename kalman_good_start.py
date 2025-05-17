import numpy as np
import pandas as pd
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import math

# --- Funkcje pomocnicze ---

def quaternion_multiply(q1, q2):
    """Mnożenie kwaternionów (w, x, y, z)"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def quaternion_conjugate(q):
    """Sprzężenie kwaternionu"""
    q_conj = np.copy(q)
    q_conj[1:] *= -1
    return q_conj

def rotate_vector_by_quaternion(v, q):
    """Obrót wektora v przez kwaternion q"""
    q_v = np.concatenate(([0], v)) # Kwaternion czysto wektorowy
    q_rotated = quaternion_multiply(quaternion_multiply(q, q_v), quaternion_conjugate(q))
    return q_rotated[1:]

def normalize_quaternion(q):
    """Normalizacja kwaternionu"""
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1.0, 0.0, 0.0, 0.0]) # Zwróć kwaternion jednostkowy
    return q / norm

def ned_to_lla(ned, ref_lla):
    """Prosta konwersja NED do LLA (przybliżona, płaska Ziemia)"""
    lat0, lon0, alt0 = np.radians(ref_lla[0]), np.radians(ref_lla[1]), ref_lla[2]
    R_earth = 6378137.0  # Promień równikowy Ziemi
    d_north, d_east, d_down = ned

    dlat = np.degrees(d_north / R_earth)
    dlon = np.degrees(d_east / (R_earth * np.cos(lat0)))

    new_lat = ref_lla[0] + dlat
    new_lon = ref_lla[1] + dlon
    new_alt = alt0 - d_down # Dół jest przeciwny do wysokości
    return np.array([new_lat, new_lon, new_alt])

# --- Definicja EKF ---

class InsEkf(ExtendedKalmanFilter):
    def __init__(self, dt_imu, initial_state, initial_cov, Q_std_devs, R_gps_pos, R_gps_vel, R_mag):
        # Stan: [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz] (16 stanów)
        dim_x = 16
        # Pomiar GPS: [pn, pe, pd, vn, ve, vd] (6 pomiarów) lub tylko pozycja [pn, pe, pd] (3 pomiary)
        # Pomiar Mag: [mx, my, mz] (3 pomiary)
        dim_z_gps = 6 # Zakładamy pozycję i prędkość z GPS
        super().__init__(dim_x=dim_x, dim_z=dim_z_gps) # dim_z ustawiane dynamicznie w update

        self.x = initial_state
        self.P = initial_cov
        self.dt_imu = dt_imu # Nominalny okres próbkowania IMU

        # Szumy procesu Q - dostrajanie jest KLUCZOWE!
        # Wariancje dla: acc, gyro, acc_bias_walk, gyro_bias_walk
        acc_std, gyro_std, acc_bias_std, gyro_bias_std = Q_std_devs

        # Uproszczone Q - zakładamy brak korelacji między grupami stanów
        # Lepsze Q wymagałoby dokładniejszego modelowania (np. Van Loan)
        self.Q = np.zeros((dim_x, dim_x))
        # Błędy prędkości spowodowane szumem akcelerometru (w przybliżeniu)
        self.Q[3:6, 3:6] = np.diag([acc_std**2] * 3) * self.dt_imu
        # Błędy orientacji spowodowane szumem żyroskopu (w przybliżeniu)
        # To jest bardzo uproszczone, dokładniejsze modele istnieją
        gyro_noise_effect = (gyro_std * self.dt_imu)**2
        self.Q[6:10, 6:10] = np.diag([gyro_noise_effect]*4) * 0.25 # Rozłożenie na 4 komponenty kwaternionu
        # Losowy spacer dryftów
        self.Q[10:13, 10:13] = np.diag([acc_bias_std**2] * 3) * self.dt_imu
        self.Q[13:16, 13:16] = np.diag([gyro_bias_std**2] * 3) * self.dt_imu
        # Dodajmy mały szum do pozycji, aby zapobiec zbyt dużej pewności
        self.Q[0:3, 0:3] = np.diag([1e-5]*3) * self.dt_imu


        # Macierze szumów pomiarowych
        self.R_gps_pos = np.diag(R_gps_pos)**2
        self.R_gps_vel = np.diag(R_gps_vel)**2
        # Połączona macierz R dla GPS (pozycja + prędkość)
        self.R_gps_combined = np.block([
            [self.R_gps_pos, np.zeros((3,3))],
            [np.zeros((3,3)), self.R_gps_vel]
        ])
        self.R_mag = np.diag(R_mag)**2

        # Stałe
        self.g_ned = np.array([0, 0, 9.81]) # Przybliżona grawitacja

        # Referencyjne pole magnetyczne w NED (Musisz dostosować do swojej lokalizacji!)
        # Możesz znaleźć wartości np. na https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        # Przykład dla Polski centralnej (wartości w nanoTeslach, ale tu używamy dowolnych jednostek, byle spójnych)
        # Wartości w uT (microTesla)
        mag_north = 20.0 # Przykład
        mag_east = 1.0   # Przykład
        mag_down = 45.0  # Przykład (Inklinacja)
        self.mag_ref_ned = np.array([mag_north, mag_east, mag_down])


    def predict(self, dt, u):
        """Krok predykcji EKF"""
        accel = u[:3]
        gyro = u[3:]

        # --- Pobierz stany ---
        pos = self.x[0:3]
        vel = self.x[3:6]
        q = normalize_quaternion(self.x[6:10]) # Normalizuj na wszelki wypadek
        b_a = self.x[10:13]
        b_g = self.x[13:16]

        # --- Popraw odczyty IMU ---
        accel_corr = accel - b_a
        gyro_corr = gyro - b_g

        # --- Model Procesu (fx) ---
        # 1. Aktualizacja orientacji (kinematyka kwaternionu)
        omega = gyro_corr
        dq_dt_w = -0.5 * (omega[0] * q[1] + omega[1] * q[2] + omega[2] * q[3])
        dq_dt_x = 0.5 * (omega[0] * q[0] + omega[1] * q[3] - omega[2] * q[2])
        dq_dt_y = 0.5 * (-omega[0] * q[3] + omega[1] * q[0] + omega[2] * q[1])
        dq_dt_z = 0.5 * (omega[0] * q[2] - omega[1] * q[1] + omega[2] * q[0])
        dq_dt = np.array([dq_dt_w, dq_dt_x, dq_dt_y, dq_dt_z])

        q_new = q + dq_dt * dt
        q_new = normalize_quaternion(q_new)

        # 2. Transformacja akceleracji do ramki NED
        # Użyj *średniej* orientacji (lub poprzedniej q dla prostoty)
        # Użycie q_new może być bardziej precyzyjne, ale komplikuje Jakobian F
        accel_ned = rotate_vector_by_quaternion(accel_corr, q) # Używamy q (poprzedni)

        # 3. Aktualizacja prędkości
        vel_new = vel + (accel_ned - self.g_ned) * dt

        # 4. Aktualizacja pozycji (prosta integracja Eulera)
        pos_new = pos + vel * dt # Używamy starej prędkości vel

        # 5. Aktualizacja biasów (model random walk)
        b_a_new = b_a
        b_g_new = b_g

        # --- Złóż nowy stan ---
        self.x[0:3] = pos_new
        self.x[3:6] = vel_new
        self.x[6:10] = q_new
        self.x[10:13] = b_a_new
        self.x[13:16] = b_g_new

        # --- Oblicz Jakobian F ---
        # To jest najbardziej złożona część EKF. Wymaga pochodnych cząstkowych fx po x.
        # Dla uproszczenia, można użyć numerycznego obliczania Jakobianu
        # lub zaimplementować analityczny (co jest żmudne).
        # Tutaj użyjemy *bardzo uproszczonego* analitycznego F.
        # Prawidłowy F jest znacznie bardziej skomplikowany!
        F = np.eye(self.dim_x)
        # dPos/dVel
        F[0:3, 3:6] = np.eye(3) * dt
        # dVel/dOrientacja (przez obrót accel_corr) - złożone, pomijamy dla uproszczenia
        # dVel/dBiasAcc - przez obrót
        Rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix() # SciPy quat format: [x, y, z, w]
        F[3:6, 10:13] = -Rot_matrix * dt
        # dOrientacja/dOrientacja (przez kinemtykę kwaternionu) - złożone, pomijamy
        # dOrientacja/dBiasGyro - przez kinemtykę kwaternionu - złożone, pomijamy

        # --- Predykcja kowariancji ---
        # Musimy użyć dokładniejszego F dla lepszych wyników!
        # Użycie uproszczonego F wpłynie na jakość estymacji.
        self.P = F @ self.P @ F.T + self.Q # Używamy self.Q zdefiniowanego w __init__

    def H_gps_jacobian(self, x):
        """Jakobian H dla pomiaru GPS (pozycja + prędkość)."""
        H = np.zeros((6, self.dim_x))
        H[0:3, 0:3] = np.eye(3) # dPos/dPos
        H[3:6, 3:6] = np.eye(3) # dVel/dVel
        return H

    def h_gps(self, x):
        """Funkcja pomiaru h dla GPS (pozycja + prędkość)."""
        return np.concatenate((x[0:3], x[3:6]))

    def H_mag_jacobian(self, x):
        """Jakobian H dla pomiaru magnetometru."""
        # Wymaga pochodnej h_mag względem kwaternionu. Bardzo złożone.
        # Można użyć numerycznej aproksymacji lub zaimplementować analitycznie.
        # Tutaj zwrócimy zero, co oznacza, że fuzja mag nie poprawi orientacji w tej uproszczonej wersji.
        # Prawidłowa implementacja jest konieczna dla dobrej estymacji heading'u.
        H = np.zeros((3, self.dim_x))
        # Należałoby obliczyć pochodne cząstkowe h_mag po qx, qy, qz, qw.
        # d(mag_body)/d(quat)
        return H

    def h_mag(self, x):
        """Funkcja pomiaru h dla magnetometru."""
        q = normalize_quaternion(x[6:10])
        # Obróć referencyjne pole NED do ramki body
        # Potrzebujemy obrotu z NED do Body, czyli q^-1
        q_conj = quaternion_conjugate(q)
        mag_body_predicted = rotate_vector_by_quaternion(self.mag_ref_ned, q_conj)
        return mag_body_predicted

# --- Ładowanie i przygotowanie danych ---
print("Ładowanie danych...")
gps_file_path = 'D:/Studia/MGR/Magisterka/data/27-04-25/pi4/gps_data_tractor_v1.csv'
imu_file_path = 'D:/Studia/MGR/Magisterka/data/27-04-25/pi4/imu_data_tractor_v1.csv'

try:
    gps_data_org = pd.read_csv(gps_file_path)
    imu_data = pd.read_csv(imu_file_path)
    print("Dane załadowane.")
except FileNotFoundError:
    print(f"Błąd: Nie znaleziono plików danych w '{gps_file_path}' lub '{imu_file_path}'.")
    print("Upewnij się, że ścieżki są poprawne i pliki istnieją.")
    exit()


# Sprawdzenie nazw kolumn
print("Kolumny GPS:", gps_data_org.columns)
print("Kolumny IMU:", imu_data.columns)

# === Konwersja czasu ===
# Użyj formatu, który pasuje do Twoich danych CSV. Przykłady:
# Jeśli czas jest w formacie 'HH:MM:SS.ffffff'
try:
    # Usuń spacje z nazw kolumn, jeśli istnieją
    gps_data_org.columns = gps_data_org.columns.str.strip()
    imu_data.columns = imu_data.columns.str.strip()

    # Zakładamy, że kolumna czasu w GPS to 'Local_Time' lub 'Timestamp'
    gps_time_col = 'Local_Time' if 'Local_Time' in gps_data_org.columns else 'Timestamp'
    imu_time_col = 'Timestamp' # Zakładamy 'Timestamp' dla IMU

    # Spróbuj przekonwertować na datetime, obsługując różne możliwe formaty
    try:
        gps_data_org['TimestampEpoch'] = pd.to_datetime(gps_data_org[gps_time_col], format='%H:%M:%S.%f').apply(lambda t: t.timestamp())
    except ValueError:
        try:
             # Spróbuj z formatem bez mikrosekund
             gps_data_org['TimestampEpoch'] = pd.to_datetime(gps_data_org[gps_time_col], format='%H:%M:%S').apply(lambda t: t.timestamp())
        except ValueError:
             # Spróbuj jako UNIX epoch
             gps_data_org['TimestampEpoch'] = pd.to_numeric(gps_data_org[gps_time_col])


    try:
        imu_data['TimestampEpoch'] = pd.to_datetime(imu_data[imu_time_col], format='%Y-%m-%d %H:%M:%S.%f').apply(lambda t: t.timestamp())
    except ValueError:
         try:
             imu_data['TimestampEpoch'] = pd.to_datetime(imu_data[imu_time_col], format='%H:%M:%S.%f').apply(lambda t: t.timestamp())
         except ValueError:
             try:
                 imu_data['TimestampEpoch'] = pd.to_numeric(imu_data[imu_time_col])
             except ValueError as e:
                 print(f"Błąd konwersji czasu IMU: {e}")
                 # Sprawdź pierwsze kilka wartości, które powodują błąd
                 print("Pierwsze kilka wartości czasu IMU:")
                 print(imu_data[imu_time_col].head())
                 exit()


    # Znajdź wspólny czas startowy (np. pierwszy czas GPS)
    # Filtrujemy dane GPS tak jak w MATLABie
    gps_data = gps_data_org.iloc[14:].copy() # Od 15 wiersza (indeks 14)
    start_time_epoch = gps_data['TimestampEpoch'].iloc[0]

    gps_data['RelTime'] = gps_data['TimestampEpoch'] - start_time_epoch
    imu_data['RelTime'] = imu_data['TimestampEpoch'] - start_time_epoch

    # Przygotowanie prędkości GPS (m/s, NED) - upewnij się, że kolumny istnieją
    if 'Speed' in gps_data.columns and 'Heading' in gps_data.columns:
       gps_data['Speed_mps'] = gps_data['Speed'] / 3.6
       # Sprawdzenie czy Heading jest w stopniach czy radianach (zakładamy stopnie)
       heading_rad = np.radians(gps_data['Heading'])
       gps_data['VelN'] = gps_data['Speed_mps'] * np.cos(heading_rad)
       gps_data['VelE'] = gps_data['Speed_mps'] * np.sin(heading_rad)
       gps_data['VelD'] = 0 # Zakładamy ruch 2D dla prędkości GPS
    else:
       print("Brak kolumn 'Speed' lub 'Heading' w danych GPS. Prędkość GPS nie będzie używana.")
       # Wypełnij zerami, jeśli nie ma danych
       gps_data['VelN'] = 0.0
       gps_data['VelE'] = 0.0
       gps_data['VelD'] = 0.0


    # === Przygotuj dane IMU ===
    # Upewnij się, że nazwy kolumn są poprawne
    accel_cols = ['LinAccel_X', 'LinAccel_Y', 'LinAccel_Z']
    gyro_cols = ['Gyro_X', 'Gyro_Y', 'Gyro_Z']
    mag_cols = ['Mag_X', 'Mag_Y', 'Mag_Z']

    if not all(col in imu_data.columns for col in accel_cols + gyro_cols + mag_cols):
        print("Błąd: Brak wymaganych kolumn IMU (LinAccel_*, Gyro_*, Mag_*)")
        print("Dostępne kolumny IMU:", imu_data.columns)
        exit()

    accel = imu_data[accel_cols].values
    gyro = imu_data[gyro_cols].values # Zakładamy radiany/s. Jeśli są w stopniach/s, przelicz!
    mag = imu_data[mag_cols].values

    # Synchronizacja czasowa jak w MATLABie
    first_good_imu_idx = imu_data[imu_data['RelTime'] >= gps_data['RelTime'].iloc[0]].index[0]
    imu_data = imu_data.iloc[first_good_imu_idx:].reset_index(drop=True)
    accel = accel[first_good_imu_idx:]
    gyro = gyro[first_good_imu_idx:]
    mag = mag[first_good_imu_idx:]

    imu_sec = imu_data['RelTime'].values
    gps_sec = gps_data['RelTime'].values

    print(f"Pierwszy znacznik czasu GPS (względny): {gps_sec[0]:.3f} s")
    print(f"Pierwszy znacznik czasu IMU po sync (względny): {imu_sec[0]:.3f} s")

except KeyError as e:
    print(f"Błąd: Brak kluczowej kolumny w danych: {e}")
    print("Sprawdź nazwy kolumn w plikach CSV.")
    exit()
except Exception as e:
    print(f"Wystąpił nieoczekiwany błąd podczas przetwarzania danych: {e}")
    exit()

# --- Połącz i posortuj wszystkie dane według czasu ---
imu_data['Type'] = 'IMU'
gps_data['Type'] = 'GPS'

# Wybierz potrzebne kolumny i zmień nazwy dla spójności
imu_subset = imu_data[['RelTime', 'Type', 'LinAccel_X', 'LinAccel_Y', 'LinAccel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Mag_X', 'Mag_Y', 'Mag_Z']].copy()
imu_subset.rename(columns={'RelTime': 'Time'}, inplace=True)

gps_subset = gps_data[['RelTime', 'Type', 'Latitude', 'Longitude', 'Altitude', 'VelN', 'VelE', 'VelD']].copy()
gps_subset.rename(columns={'RelTime': 'Time', 'Altitude': 'Alt'}, inplace=True) # Używamy 'Alt' dla spójności

# Połącz ramki danych
all_data = pd.concat([imu_subset, gps_subset], ignore_index=True)

# Sortuj według czasu
all_data.sort_values(by='Time', inplace=True)
all_data.reset_index(drop=True, inplace=True)

print("Połączone i posortowane dane:")
print(all_data.head())
print(all_data.tail())


# --- Inicjalizacja Filtru ---
print("Inicjalizacja EKF...")
# Częstotliwość IMU
imu_fs = 100 # Hz
dt_imu = 1.0 / imu_fs

# Początkowa pozycja (LLA) i konwersja do NED (0,0,0)
ref_lla = [gps_data['Latitude'].iloc[0], gps_data['Longitude'].iloc[0], gps_data['Alt'].iloc[0] if 'Alt' in gps_data.columns else 0]
print(f"Referencyjne LLA: {ref_lla}")

# Początkowy stan [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz]
initial_state = np.zeros(16)
initial_state[0:3] = [0, 0, 0]  # Początkowa pozycja NED
initial_state[3:6] = [gps_data['VelN'].iloc[0], gps_data['VelE'].iloc[0], gps_data['VelD'].iloc[0]] # Początkowa prędkość z GPS
initial_state[6] = 1.0      # Początkowy kwaternion (qw=1, brak obrotu)
# Początkowe biasy = 0
print(f"Stan początkowy: {initial_state}")

# Początkowa kowariancja P - odzwierciedla niepewność początkowego stanu
initial_cov = np.diag([
    1.0, 1.0, 1.0,       # Niepewność pozycji (m^2)
    0.5, 0.5, 0.5,       # Niepewność prędkości ((m/s)^2)
    0.1, 0.1, 0.1, 0.1,  # Niepewność orientacji (rad^2) - trudna do określenia
    1e-3, 1e-3, 1e-3,    # Niepewność biasu akcelerometru ((m/s^2)^2)
    1e-4, 1e-4, 1e-4     # Niepewność biasu żyroskopu ((rad/s)^2)
])

# Szumy procesu Q - STANDARDOWE ODCHYLENIA (nie wariancje!) - DO STROJENIA!
# [acc_std, gyro_std, acc_bias_std, gyro_bias_std]
# Te wartości mają OGROMNY wpływ na wynik! Zacznij od wartości z datasheet IMU jeśli masz.
Q_std_devs = [
    5e1,  # Szum akcelerometru (m/s^2 / sqrt(Hz)) - DOSTROJ!
    1e-2,  # Szum żyroskopu (rad/s / sqrt(Hz)) - DOSTROJ!
    1e-3,  # Random walk biasu akcelerometru (m/s^3 / sqrt(Hz)) - DOSTROJ!
    1e-5   # Random walk biasu żyroskopu (rad/s^2 / sqrt(Hz)) - DOSTROJ!
]

# Szumy pomiarowe R - STANDARDOWE ODCHYLENIA (nie wariancje!)
# Zgodne z Twoim kodem MATLAB
RTK_FIXED_POS_STD_HORIZ = 0.01 # [m]
RTK_FIXED_POS_STD_VERT = 0.01  # [m]
RTK_FIXED_VEL_STD      = 0.03  # [m/s]
MAG_STD = 0.1                  # Niepewność magnetometru (jednostki jak mag_ref_ned) - DOSTROJ!

R_gps_pos = [RTK_FIXED_POS_STD_HORIZ, RTK_FIXED_POS_STD_HORIZ, RTK_FIXED_POS_STD_VERT]
R_gps_vel = [RTK_FIXED_VEL_STD, RTK_FIXED_VEL_STD, RTK_FIXED_VEL_STD]
R_mag = [MAG_STD, MAG_STD, MAG_STD]

# Utwórz instancję filtru
ekf = InsEkf(dt_imu, initial_state, initial_cov, Q_std_devs, R_gps_pos, R_gps_vel, R_mag)

# --- Główna pętla filtru ---
print("Rozpoczynanie filtrowania...")
num_samples = len(all_data)
est_pos_ned = np.zeros((num_samples, 3))
est_vel_ned = np.zeros((num_samples, 3))
est_orient_quat = np.zeros((num_samples, 4))
timestamps = np.zeros(num_samples)

last_time = all_data['Time'].iloc[0]

# Zmienne do przechowywania ostatniego odczytu IMU dla predykcji
last_imu_reading = np.zeros(6) # [ax, ay, az, gx, gy, gz]

# --- Zapis wyników ---
# Chcemy zapisywać stan z częstotliwością IMU (lub zbliżoną)
output_times = []
output_pos = []
output_vel = []
output_orient = []

for i in range(num_samples):
    row = all_data.iloc[i]
    current_time = row['Time']
    data_type = row['Type']

    # --- Krok predykcji ---
    # Wykonaj predykcję od ostatniego czasu do bieżącego czasu pomiaru
    dt = current_time - last_time
    if dt < 0:
      print(f"Ostrzeżenie: Ujemny krok czasowy dt={dt} przy indeksie {i}. Pomijanie kroku.")
      last_time = current_time # Zaktualizuj czas, aby uniknąć problemów w przyszłości
      continue
    elif dt > 0.5: # Ostrzeżenie o dużej przerwie
        print(f"Ostrzeżenie: Duży krok czasowy dt={dt:.3f}s przy indeksie {i} ({data_type}).")


    if dt > 1e-9 : # Tylko jeśli czas się zmienił
      # Użyj ostatniego dostępnego odczytu IMU do predykcji w interwałach
      # Można by interpolować IMU, ale dla prostoty użyjemy ostatniego
      # W bardziej zaawansowanych implementacjach, predict może być wołany
      # wielokrotnie z mniejszym dt_pred = dt / N
      if np.any(last_imu_reading): # Upewnij się, że mamy jakiekolwiek dane IMU
          ekf.predict(dt=dt, u=last_imu_reading)
      else:
          # Jeśli nie ma jeszcze danych IMU, tylko propaguj czas w kowariancji (uproszczenie)
           ekf.P += ekf.Q # Bardzo duże uproszczenie - stan się nie zmienia, tylko niepewność rośnie
           pass # Nie można przewidzieć bez danych IMU

    # --- Zapisz estymowany stan (po predykcji) ---
    # Zapisujemy stan *przed* ewentualną aktualizacją pomiarem
    # aby mieć gęste wyniki czasowe odpowiadające krokom predykcji
    timestamps[i] = current_time
    est_pos_ned[i, :] = ekf.x[0:3]
    est_vel_ned[i, :] = ekf.x[3:6]
    est_orient_quat[i, :] = ekf.x[6:10]

    # --- Krok aktualizacji (jeśli to pomiar GPS lub Mag) ---
    if data_type == 'IMU':
        # Zapisz odczyt IMU do użycia w następnej predykcji
        accel = row[['LinAccel_X', 'LinAccel_Y', 'LinAccel_Z']].values
        gyro = row[['Gyro_X', 'Gyro_Y', 'Gyro_Z']].values
        mag = row[['Mag_X', 'Mag_Y', 'Mag_Z']].values # Zachowaj Mag, jeśli chcesz go użyć
        last_imu_reading = np.concatenate((accel, gyro))

        # Opcjonalnie: Fuzja magnetometru (jeśli Jakobian H_mag jest zaimplementowany poprawnie!)
        # if np.linalg.norm(mag) > 1e-6: # Sprawdź czy odczyt nie jest zerowy
        #     try:
        #         # UWAGA: Wymaga poprawnych H_mag_jacobian i h_mag
        #         # R_mag musi być też dostrojone
        #         ekf.update(mag, R=ekf.R_mag, HJacobian=ekf.H_mag_jacobian, Hx=ekf.h_mag)
        #         # print(f"Fused Mag at {current_time:.3f}")
        #     except Exception as e:
        #         print(f"Błąd podczas fuzji magnetometru: {e}")
        pass # W wersji uproszczonej nie fuzujemy Mag jako osobnego kroku

    elif data_type == 'GPS':
        # Przygotuj wektor pomiaru GPS
        # Konwersja LLA do NED względem punktu referencyjnego
        lat, lon = row['Latitude'], row['Longitude']
        alt = row['Alt'] if 'Alt' in row else ref_lla[2] # Użyj ref alt jeśli brak
        
        # Prosta konwersja - dla większych odległości potrzebna lepsza metoda
        R_earth = 6378137.0
        pos_ned_meas = np.array([
            np.radians(lat - ref_lla[0]) * R_earth,
            np.radians(lon - ref_lla[1]) * R_earth * np.cos(np.radians(ref_lla[0])),
            -(alt - ref_lla[2]) # Dół jest przeciwny do wysokości
        ])
        vel_ned_meas = row[['VelN', 'VelE', 'VelD']].values

        z_gps = np.concatenate((pos_ned_meas, vel_ned_meas))

        try:
            ekf.update(z_gps, R=ekf.R_gps_combined, HJacobian=ekf.H_gps_jacobian, Hx=ekf.h_gps)
            print(f"Fused GPS at {current_time:.3f} (Index {i})")
        except Exception as e:
             print(f"Błąd podczas fuzji GPS (indeks {i}): {e}")
             # Można tu dodać logowanie stanu/kowariancji dla debugowania
             # print("Stan x:", ekf.x)
             # print("Kowariancja P:", np.diag(ekf.P)) # Diagonal dla czytelności


    # Zaktualizuj czas
    last_time = current_time

print("Filtrowanie zakończone.")

# --- Wyniki i Wizualizacja ---
print("Przygotowanie wyników...")

# Usuń potencjalne początkowe próbki, gdzie czas był stały
valid_indices = np.where(np.diff(timestamps, prepend=timestamps[0]) > 1e-9)[0]
if len(valid_indices) < len(timestamps):
    print(f"Usunięto {len(timestamps) - len(valid_indices)} duplikatów czasowych.")
    timestamps = timestamps[valid_indices]
    est_pos_ned = est_pos_ned[valid_indices]
    est_vel_ned = est_vel_ned[valid_indices]
    est_orient_quat = est_orient_quat[valid_indices]


# Oblicz prędkość wypadkową
est_speed = np.linalg.norm(est_vel_ned[:, 0:2], axis=1) # Prędkość horyzontalna
gps_speed = gps_data['Speed_mps'].values # Używamy już przeliczonej

# Konwersja pozycji NED z powrotem do LLA dla porównania
est_pos_lla = np.array([ned_to_lla(ned, ref_lla) for ned in est_pos_ned])

print("Rysowanie wyników...")

# 1. Trajektoria
plt.figure(figsize=(10, 8))
plt.plot(gps_data['Longitude'], gps_data['Latitude'], 'r.-', label='GPS Raw', markersize=4)
plt.plot(est_pos_lla[:, 1], est_pos_lla[:, 0], 'b-', label='EKF Estimate')
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Trajektoria GPS vs EKF")
plt.legend()
plt.grid(True)
plt.axis('equal')

# 2. Prędkość
plt.figure(figsize=(12, 6))
plt.plot(timestamps, est_speed, 'b-', label='EKF Speed', linewidth=1)
plt.plot(gps_sec, gps_speed, 'ro', label='GPS Speed', markersize=4)

# Opcjonalnie: Średnia krocząca dla wygładzenia EKF speed (jak w MATLAB)
window_len_sec = 2.0 # Długość okna w sekundach
window_len_samples = int(window_len_sec * imu_fs)
if len(est_speed) > window_len_samples:
    est_speed_smoothed = np.convolve(est_speed, np.ones(window_len_samples)/window_len_samples, mode='valid')
    # Dopasuj oś czasu dla wygładzonych danych
    smoothed_timestamps = timestamps[window_len_samples//2 : -window_len_samples//2 + 1]
    if len(smoothed_timestamps) == len(est_speed_smoothed): # Upewnij się, że rozmiary pasują
      plt.plot(smoothed_timestamps, est_speed_smoothed, 'g-', label=f'EKF Smoothed ({window_len_sec}s window)', linewidth=1.5)


plt.xlabel("Czas (s)")
plt.ylabel("Prędkość (m/s)")
plt.title("Estymowana prędkość vs Prędkość GPS")
plt.legend()
plt.grid(True)
plt.ylim(bottom=0) # Prędkość nie powinna być ujemna

# 3. Komponenty prędkości (opcjonalnie)
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(timestamps, est_vel_ned[:, 0], 'b-', label='EKF Vel North')
plt.plot(gps_sec, gps_data['VelN'], 'ro', label='GPS Vel North')
plt.legend()
plt.grid(True)
plt.ylabel("Vel N (m/s)")
plt.title("Komponenty prędkości NED")

plt.subplot(3, 1, 2)
plt.plot(timestamps, est_vel_ned[:, 1], 'b-', label='EKF Vel East')
plt.plot(gps_sec, gps_data['VelE'], 'ro', label='GPS Vel East')
plt.legend()
plt.grid(True)
plt.ylabel("Vel E (m/s)")

plt.subplot(3, 1, 3)
plt.plot(timestamps, est_vel_ned[:, 2], 'b-', label='EKF Vel Down')
# plt.plot(gps_sec, gps_data['VelD'], 'ro', label='GPS Vel Down') # Zwykle VelD z GPS jest mało wiarygodne
plt.legend()
plt.grid(True)
plt.ylabel("Vel D (m/s)")
plt.xlabel("Czas (s)")

plt.tight_layout()
plt.show()

print("Gotowe.")