import time
import board
import busio
import adafruit_adxl34x
import threading

class IMUReader(threading.Thread):
    def __init__(self, sampling_frequency=200, output_frequency=10, window_size=10):
        threading.Thread.__init__(self)
        self.sampling_frequency = sampling_frequency
        self.output_frequency = output_frequency
        self.decimation_factor = int(self.sampling_frequency / self.output_frequency)
        self.window_size = window_size
        self.acceleration_history_x = []
        self.acceleration_history_y = []
        self.acceleration_history_z = []
        self.sample_counter = 0
        self.latest_data = None
        self.running = True
        self._initialize_imu()

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

    def get_latest_data(self):
        return self.latest_data

    def run(self):
        try:
            while self.running:
                raw_data, filtered_data = self.read_imu_data()
                raw_x, raw_y, raw_z = raw_data
                filtered_x, filtered_y, filtered_z = filtered_data

                if self.sample_counter % self.decimation_factor == 0:
                    self.latest_data = {
                        "raw": (raw_x, raw_y, raw_z),
                        "filtered": (filtered_x, filtered_y, filtered_z),
                    }
                time.sleep(1.0 / self.sampling_frequency)
        except KeyboardInterrupt:
            print("Zatrzymywanie wątku IMU...")
            self.running = False
        except Exception as e:
            print(f"Wystąpił błąd w wątku IMU: {e}")
        finally:
            print("Zakończono wątek IMU.")

if __name__ == '__main__':
    imu_reader = IMUReader()
    imu_reader.start()
    try:
        while True:
            data = imu_reader.get_latest_data()
            if data:
                print(data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        imu_reader.running = False
        imu_reader.join()