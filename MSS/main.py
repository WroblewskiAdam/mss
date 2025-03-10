import time
import threading
from GpsRTK import GpsRTK
# from imu_sensor import IMUSensor  # Załóżmy, że masz moduł do obsługi IMU

def read_gps(gps):
    """Czyta dane GNSS co 1 sekundę"""
    while True:
        gnss_data = next(gps.read_gnss_data(), None)
        if gnss_data:
            print("GNSS:", gnss_data)
        # time.sleep(1)  # 1 Hz

def read_imu(imu):
    """Czyta dane z IMU co 0.1 sekundy"""
    while True:
        imu_data = imu.read_data()
        print("IMU:", imu_data)
        time.sleep(0.1)  # 10 Hz

def main():
    gps = GpsRTK(
        serial_port='/dev/ttyUSB0',
        baudrate=115200,
        caster='system.asgeupos.pl',
        port=8080,
        mountpoint='/RTN4G_VRS_RTCM32',
        user='pwmgr/adamwrb:Globus7142001'
    )
    
    # imu = IMUSensor()  # Przykładowa inicjalizacja IMU
    
    gps.connect_ntrip()

    # Uruchomienie wątków dla GPS i IMU
    gps_thread = threading.Thread(target=read_gps, args=(gps,), daemon=True)
    # imu_thread = threading.Thread(target=read_imu, args=(imu,), daemon=True)
    
    gps_thread.start()
    # imu_thread.start()

    try:
        while True:
            time.sleep(1)  # Główna pętla, aby uniknąć zakończenia programu
    except KeyboardInterrupt:
        print("Zamykanie...")
        gps.close()

if __name__ == "__main__":
    main()
