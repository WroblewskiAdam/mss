import threading
import time
import csv
import os
from datetime import datetime
import sys
sys.path.append("/home/pi/mss")
from GPS.GpsRtkClient import GPSrtk  # Zaimportuj klasę GPSrtk z GPSrtk.py
from IMU.ImuReader import IMUReader  # Zaimportuj klasę IMUReader z imu2.py

class DataCollector:
    def __init__(self, gps_config, imu_config, base_path='/home/pi/mss/data/', filename='combined_data.csv'):
        self.gps_config = gps_config
        self.imu_config = imu_config
        self.base_path = base_path
        self.filename = filename
        self.gps_data = None
        self.gps_object = None # Dodajemy atrybut dla obiektu GPSrtk
        self.imu_data_raw = None
        self.imu_data_filtered = None
        self.data_lock = threading.Lock()  # Zabezpieczenie dostępu do danych
        self.running = True

    def gps_thread(self):
        """Wątek odczytujący dane z GPS RTK."""
        self.gps_object = GPSrtk(**self.gps_config)  # Zapisujemy instancję GPSrtk w atrybucie
        if not self.gps_object.connect_ntrip():
            self.stop()
            return
        if not self.gps_object.connect_gnss():
            self.stop()
            return

        while self.running:
            try:
                if self.gps_object.serial_com:
                    raw_data, parsed_data = self.gps_object.nmr.read()
                    if b"GNGGA" in raw_data:
                        RTCM_response = self.gps_object.send_gga_to_ntrip(raw_data.decode())
                        self.gps_object.serial_com.write(RTCM_response)
                        with self.data_lock:  # Zabezpiecz dostęp do współdzielonych danych
                            self.gps_data = parsed_data
                    if b"GNVTG" in raw_data:
                        with self.data_lock:
                            self.gps_object.VTGdata = parsed_data

            except Exception as e:
                print(f"GPS thread error: {e}")
                time.sleep(2)

    def imu_thread(self):
        """Wątek odczytujący dane z IMU."""
        imu = IMUReader(**self.imu_config)
        while self.running:
            try:
                raw_data, filtered_data = imu.read_imu_data()
                with self.data_lock:  # Zabezpiecz dostęp do współdzielonych danych
                    self.imu_data_raw = raw_data
                    self.imu_data_filtered = filtered_data
                time.sleep(1.0 / imu.sampling_frequency)  # Dostosuj opóźnienie do częstotliwości próbkowania IMU
            except Exception as e:
                print(f"IMU thread error: {e}")
                time.sleep(2)

    def save_thread(self):
        """Wątek zapisujący dane do pliku."""
        while self.running:
            try:
                with self.data_lock:  # Zabezpiecz dostęp do współdzielonych danych
                    if self.gps_data and self.imu_data_raw and self.imu_data_filtered:
                        # Tworzenie ścieżki z aktualną datą
                        current_date = datetime.now().strftime('%Y-%m-%d')
                        data_dir = os.path.join(self.base_path, current_date)
                        # Tworzenie folderu jeśli nie istnieje
                        os.makedirs(data_dir, exist_ok=True)
                        # Pełna ścieżka do pliku
                        file_path = os.path.join(data_dir, self.filename)

                        with open(file_path, 'a', newline='') as file:
                            writer = csv.writer(file)
                            if file.tell() == 0:
                                writer.writerow([
                                    "Time", "Latitude", "Longitude", "Speed", "Quality",
                                    "IMU Raw X", "IMU Raw Y", "IMU Raw Z",
                                    "IMU Filtered X", "IMU Filtered Y", "IMU Filtered Z"
                                ])
                            writer.writerow([
                                self.gps_data.time, f"{self.gps_data.lat:.8f}", f"{self.gps_data.lon:.8f}", self.gps_object.VTGdata.sogk, self.gps_data.quality,
                                *self.imu_data_raw, *self.imu_data_filtered
                            ])
                        self.gps_data = None  # Zresetuj dane po zapisie
                        self.imu_data_raw = None
                        self.imu_data_filtered = None
            except Exception as e:
                print(f"Save thread error: {e}")
            time.sleep(1)  # Zapisuj dane co sekundę

    def start(self):
        """Uruchomienie wątków."""
        self.gps_thread_var = threading.Thread(target=self.gps_thread)
        self.imu_thread_var = threading.Thread(target=self.imu_thread)
        self.save_thread_var = threading.Thread(target=self.save_thread)

        self.gps_thread_var.start()
        self.imu_thread_var.start()
        self.save_thread_var.start()

    def stop(self):
        """Zatrzymanie wątków."""
        self.running = False
        self.gps_thread_var.join()
        self.imu_thread_var.join()
        self.save_thread_var.join()
        print("Wszystkie wątki zostały zakończone.")

if __name__ == "__main__":
    gps_config = {
        'serial_port': '/dev/ttyUSB0',
        'baudrate': 115200,
        'caster': 'system.asgeupos.pl',
        'port': 8080,
        'mountpoint': '/RTN4G_VRS_RTCM32',
        'user': 'pwmgr/adamwrb:Globus7142001',
        'save_file': False,  # Zapisywanie do pliku obsługiwane w głównym skrypcie
    }
    imu_config = {
        'sampling_frequency': 100,
        'output_frequency': 10,
        'window_size': 10,
    }
    data_collector = DataCollector(gps_config, imu_config)

    try:
        data_collector.start()
        while True:  # Utrzymuj główny wątek przy życiu
            time.sleep(1)
    except KeyboardInterrupt:
        print("Zatrzymywanie zbierania danych...")
        data_collector.stop()