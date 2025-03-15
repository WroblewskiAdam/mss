import threading
import time
import csv
import sys
import os
from datetime import datetime

sys.path.append("/home/pi/mss")
from lc29h_gps_rtk_hat_code.python.rtk_rover.GPSrtk import GPSrtk
from IMU.imu_reader import IMUReader


def main():
    # Inicjalizacja GPS
    gps_client = GPSrtk(
        serial_port='/dev/ttyUSB0',
        baudrate=115200,
        caster='system.asgeupos.pl',
        port=8080,
        mountpoint='/RTN4G_VRS_RTCM32',
        user='pwmgr/adamwrb:Globus7142001',
        save_file=True,
    )

    # Inicjalizacja IMU
    imu_reader = IMUReader(sampling_frequency=200, output_frequency=10, window_size=20)

    # Uruchomienie wątków
    gps_client.start()
    imu_reader.start()

    # Konfiguracja ścieżki i nazwy pliku
    base_path = "/home/pi/mss/data"
    date_folder_name = datetime.now().strftime("%Y-%m-%d")
    date_folder_path = os.path.join(base_path, date_folder_name)
    
    file_name = "dupa.csv"
    full_path = os.path.join(date_folder_path, file_name)

    # Utworzenie folderu z datą, jeśli nie istnieje
    if not os.path.exists(date_folder_path):
        try:
            os.makedirs(date_folder_path)
            print(f"Utworzono folder: {date_folder_path}")
        except OSError as e:
            print(f"Błąd podczas tworzenia folderu: {e}")
            return  # Zakończ program, jeśli nie można utworzyć folderu

    csv_file = None  # Inicjalizacja zmiennej
    try:
        # Otwarcie pliku CSV
        csv_file = open(full_path, 'a', newline='')
        csv_writer = csv.writer(csv_file)

        # Zapis nagłówka CSV (tylko raz)
        csv_writer.writerow(['Timestamp', 'GPS Time', 'Lat', 'Lon', 'Speed', 'Quality',
                             'Raw X', 'Raw Y', 'Raw Z', 'Filtered X', 'Filtered Y', 'Filtered Z'])

        while True:
            # Pobranie timestamp
            now = datetime.now()
            timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Format YYYY-MM-DD HH:MM:SS.mmm

            # Odczyt danych z GPS
            gps_data = gps_client.get_latest_data()
            if gps_data:
                print(f"{timestamp} GPS: Time={gps_data['time']}, Lat={gps_data['lat']:.8f}, Lon={gps_data['lon']:.8f}, Speed={gps_data['speed']} km/s, Quality={gps_data['quality']}")

            # Odczyt danych z IMU
            imu_data = imu_reader.get_latest_data()
            if imu_data:
                raw_x, raw_y, raw_z = imu_data['raw']
                filtered_x, filtered_y, filtered_z = imu_data['filtered']
                print(f"{timestamp} IMU: Raw X={raw_x:.2f}, Y={raw_y:.2f}, Z={raw_z:.2f} m/s^2 | Filtered X={filtered_x:.2f}, Y={filtered_y:.2f}, Z={filtered_z:.2f} m/s^2")

            # Zapis do CSV
            try:
                if gps_data and imu_data:
                    lat_str = f"{gps_data['lat']:.10f}"
                    lon_str = f"{gps_data['lon']:.10f}"
                    speed_str = f"{gps_data['speed']:.3f}"
                    raw_x_str = f"{imu_data['raw'][0]:.4f}"
                    raw_y_str = f"{imu_data['raw'][1]:.4f}"
                    raw_z_str = f"{imu_data['raw'][2]:.4f}"
                    filtered_x_str = f"{imu_data['filtered'][0]:.4f}"
                    filtered_y_str = f"{imu_data['filtered'][1]:.4f}"
                    filtered_z_str = f"{imu_data['filtered'][2]:.4f}"

                    csv_writer.writerow([
                        timestamp,
                        gps_data['time'],
                        lat_str,
                        lon_str,
                        speed_str,
                        gps_data['quality'],
                        raw_x_str,
                        raw_y_str,
                        raw_z_str,
                        filtered_x_str,
                        filtered_y_str,
                        filtered_z_str
                    ])
                elif gps_data:
                    lat_str = f"{gps_data['lat']:.10f}"
                    lon_str = f"{gps_data['lon']:.10f}"
                    speed_str = f"{gps_data['speed']:.3f}"

                    csv_writer.writerow([
                        timestamp,
                        gps_data['time'],
                        lat_str,
                        lon_str,
                        speed_str,
                        gps_data['quality'],
                        '', '', '', '', '', ''  # Puste wartości dla IMU
                    ])
                elif imu_data:
                    raw_x_str = f"{imu_data['raw'][0]:.4f}"
                    raw_y_str = f"{imu_data['raw'][1]:.4f}"
                    raw_z_str = f"{imu_data['raw'][2]:.4f}"
                    filtered_x_str = f"{imu_data['filtered'][0]:.4f}"
                    filtered_y_str = f"{imu_data['filtered'][1]:.4f}"
                    filtered_z_str = f"{imu_data['filtered'][2]:.4f}"

                    csv_writer.writerow([
                        timestamp,
                        '',  # Puste wartości dla GPS
                        '',
                        '',
                        '',
                        '',
                        raw_x_str,
                        raw_y_str,
                        raw_z_str,
                        filtered_x_str,
                        filtered_y_str,
                        filtered_z_str
                    ])
                else:
                    csv_writer.writerow([timestamp, '', '', '', '', '', '', '', '', '', '', ''])


            except Exception as e:
                print(f"Błąd zapisu do CSV: {e}")

            time.sleep(0.1)  # Odczyt co 100ms

    except KeyboardInterrupt:
        print("Zatrzymywanie programu...")
    finally:
        gps_client.running = False
        imu_reader.running = False
        gps_client.join()
        imu_reader.join()
        print("Program zakończony.")
        if csv_file:
            csv_file.close()  # Zawsze zamknij plik
        print("Plik CSV został zamknięty.")


if __name__ == "__main__":
    main()