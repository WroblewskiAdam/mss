import time
import board
import busio
import adafruit_adxl34x
import csv
import os
from datetime import datetime
import socket

class IMUReader:
    def __init__(self, sampling_frequency=200, output_frequency=10, window_size=20, save_file=True, base_path='/home/pi/mss/data/', filename='imu_data.csv'):
        self.sampling_frequency = sampling_frequency
        self.output_frequency = output_frequency
        self.window_size = window_size
        self.acceleration_history_x = []
        self.acceleration_history_y = []
        self.acceleration_history_z = []
        self.sample_counter = 0
        self._initialize_imu()

        self.save_file = save_file
        self.base_path = base_path
        self.filename = filename

        self.output_period = 1.0 / self.output_frequency  # Interwał między wyjściami
        self.next_output_time = time.perf_counter() # Kiedy następny output

        self.hostname = socket.gethostname() # Pobranie nazwy hosta

    def _initialize_imu(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.accelerometer = adafruit_adxl34x.ADXL345(i2c)
            self.accelerometer.data_rate = adafruit_adxl34x.DataRate.RATE_200_HZ
            self.accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G
        except ValueError as e:
            print(f"Błąd inicjalizacji ADXL345: {e}")
            print("Upewnij się, że ADXL345 jest poprawnie podłączony i adres I2C jest prawidłowy.")
            exit()

    def _moving_average_filter(self, history, new_value):
        history.append(new_value)
        if len(history) > self.window_size:
            history.pop(0)
        return sum(history) / len(history) if history else 0

    def read_imu_data(self):
        raw_x, raw_y, raw_z = self.accelerometer.acceleration
        filtered_x = self._moving_average_filter(self.acceleration_history_x, raw_x)
        filtered_y = self._moving_average_filter(self.acceleration_history_y, raw_y)
        filtered_z = self._moving_average_filter(self.acceleration_history_z, raw_z)
        self.sample_counter += 1
        return (raw_x, raw_y, raw_z), (filtered_x, filtered_y, filtered_z)

    def _append_to_file(self, raw_x, raw_y, raw_z, filtered_x, filtered_y, filtered_z):
        if self.save_file:
            try:
                # Tworzenie ścieżki z aktualną datą
                current_date = datetime.now().strftime('%Y-%m-%d')
                date_dir = os.path.join(self.base_path, current_date)

                # Tworzenie podfolderu z nazwą hosta
                hostname_dir = os.path.join(date_dir, self.hostname)

                # Tworzenie folderów jeśli nie istnieją
                os.makedirs(hostname_dir, exist_ok=True)

                # Pełna ścieżka do pliku
                file_path = os.path.join(hostname_dir, self.filename)

                # Pobierz aktualny czas lokalny (tylko godzina z dokładnością do tysięcznej sekundy)
                local_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                with open(file_path, 'a', newline='') as file:
                    writer = csv.writer(file)
                    if file.tell() == 0:
                        writer.writerow(["Local Time", "Raw X", "Raw Y", "Raw Z", "Filtered X", "Filtered Y", "Filtered Z"])
                    writer.writerow([
                        local_time,
                        f"{raw_x:.4f}",  # Formatowanie do 4 miejsc po przecinku
                        f"{raw_y:.4f}",
                        f"{raw_z:.4f}",
                        f"{filtered_x:.4f}",
                        f"{filtered_y:.4f}",
                        f"{filtered_z:.4f}"
                    ])
            except Exception as e:
                print(f"Błąd przy zapisie do pliku: {e}")

    def run_loop(self): # Metoda testowa do uruchomienia pętli z imu.py, do weryfikacji poprawności klasy
        try:
            while True:
                raw_data, filtered_data = self.read_imu_data()
                raw_x, raw_y, raw_z = raw_data
                filtered_x, filtered_y, filtered_z = filtered_data

                current_time = time.perf_counter()
                if current_time >= self.next_output_time:
                    # Pobierz aktualny czas lokalny (tylko godzina z dokładnością do tysięcznej sekundy)
                    local_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    print(f"Time: {local_time}, R: X={raw_x:.2f} Y={raw_y:.2f} Z={raw_z:.2f} \t F: X={filtered_x:.2f} Y={filtered_y:.2f} Z={filtered_z:.2f}")
                    self._append_to_file(raw_x, raw_y, raw_z, filtered_x, filtered_y, filtered_z)  # Zapis do pliku

                    self.next_output_time += self.output_period  # Ustal następny czas wyjścia

                # Krótkie uśpienie, aby zwolnić zasoby CPU
                time.sleep(0.001) #1 ms
        except KeyboardInterrupt:
            print("Program zakończony przez użytkownika.")
        except Exception as e:
            print(f"Wystąpił błąd: {e}")
        finally:
            print("Koniec programu.")


if __name__ == '__main__':
    imu_reader = IMUReader()
    imu_reader.run_loop()