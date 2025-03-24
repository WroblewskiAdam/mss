import threading
import time

import sys
sys.path.append("/home/pi/mss")
from lc29h_gps_rtk_hat_code.python.rtk_rover.GPSrtk import GPSrtk
from mss.IMU.IMUreader_threading import IMUReader


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

    try:
        while True:
            # Odczyt danych z GPS
            gps_data = gps_client.get_latest_data()
            if gps_data:
                print(f"GPS: Time={gps_data['time']}, Lat={gps_data['lat']:.8f}, Lon={gps_data['lon']:.8f}, Speed={gps_data['speed']} km/s, Quality={gps_data['quality']}")

            # Odczyt danych z IMU
            imu_data = imu_reader.get_latest_data()
            if imu_data:
                raw_x, raw_y, raw_z = imu_data['raw']
                filtered_x, filtered_y, filtered_z = imu_data['filtered']
                print(f"IMU: Raw X={raw_x:.2f}, Y={raw_y:.2f}, Z={raw_z:.2f} m/s^2 | Filtered X={filtered_x:.2f}, Y={filtered_y:.2f}, Z={filtered_z:.2f} m/s^2")

            time.sleep(0.1)  # Odczyt co 100ms
    except KeyboardInterrupt:
        print("Zatrzymywanie programu...")
        gps_client.running = False
        imu_reader.running = False
        gps_client.join()
        imu_reader.join()
        print("Program zakończony.")

if __name__ == "__main__":
    main()