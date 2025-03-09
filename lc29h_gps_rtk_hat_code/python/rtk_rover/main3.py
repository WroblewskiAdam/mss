import socket
import sys
import datetime
import base64
import time
import csv
import serial
from pynmeagps import NMEAReader, NMEAMessage

version = 0.3
useragent = f"NTRIP JCMBsoftPythonClient/{version:.1f}"

factor = 2
maxReconnect = 5
maxReconnectTime = 1200
sleepTime = 5

class NtripClient:
    def __init__(self, buffer_size=1024, user="", port=2101, caster="", mountpoint="", verbose=False, max_connect_time=0, save_to_file=False, file_path="data.csv", serial_port='/dev/ttyUSB0', serial_baudrate=115200):
        self.buffer_size = buffer_size
        self.user = base64.b64encode(user.encode()).decode("utf-8")
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.verbose = verbose
        self.max_connect_time = max_connect_time
        self.save_to_file = save_to_file
        self.file_path = file_path
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self.socket = None
        self.stream = None
        self.nmr = None
        self.initialize_serial()

    def initialize_serial(self):
        try:
            self.stream = serial.Serial(self.serial_port, self.serial_baudrate, timeout=3)
            self.nmr = NMEAReader(self.stream)
            print(f"Połączono z portem szeregowym: {self.serial_port} z prędkością {self.serial_baudrate}")
        except serial.SerialException as e:
            print(f"Błąd podczas otwierania portu szeregowego: {e}")
            sys.exit(1)

    def get_mountpoint_bytes(self):
        request = f"GET {self.mountpoint} HTTP/1.1\r\nUser-Agent: {useragent}\r\nAuthorization: Basic {self.user}\r\n\r\n"
        if self.verbose:
            print(request)
        return request.encode('ascii')

    def get_gga_bytes(self):
        while True:
            try:
                raw_data, parsed_data = self.nmr.read()
                if raw_data and b"GNGGA" in raw_data:
                    return raw_data
            except Exception as e:
                print(f"Błąd podczas odczytu z portu szeregowego: {e}")
                time.sleep(0.1)

    def read_data(self):
        reconnect_try, sleep_time = 1, sleepTime
        while reconnect_try <= maxReconnect:
            print(f'Próba połączenia {reconnect_try} z {maxReconnect}')

            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.caster, self.port))
                self.socket.settimeout(10)  # Zwiększony timeout
                print(f"Połączono z {self.caster}:{self.port}")

                self.socket.sendall(self.get_mountpoint_bytes())
                if not self.process_headers():
                    self.cleanup_connection()
                    reconnect_try += 1
                    continue

                print("Pobieranie danych...")
                self.process_data()  # Przeniesiono do oddzielnej funkcji
                self.cleanup_connection()
                return  # Wyjście z pętli po udanym połączeniu i przetworzeniu danych

            except (socket.timeout, socket.error) as e:
                print(f"Błąd połączenia: {e}")
                self.cleanup_connection()
                if reconnect_try < maxReconnect:
                    print(f"Próba ponownego połączenia za {sleep_time} sekund...")
                    time.sleep(sleep_time)
                    sleep_time = min(sleep_time * factor, maxReconnectTime)
                reconnect_try += 1

        print("Przekroczono maksymalną liczbę prób połączeń.")

    def process_headers(self):
        response = b""
        try:
            while True:
                chunk = self.socket.recv(4096)
                if not chunk:
                    break
                response += chunk
                if b"\r\n\r\n" in response:
                    break

            response_str = response.decode('utf-8')
            header_lines = response_str.split("\r\n")

            for line in header_lines:
                if "SOURCETABLE" in line:
                    print("Punkt montowania nie istnieje.")
                    return False
                elif "401 Unauthorized" in line:
                    print("Nieautoryzowany dostęp.")
                    return False
                elif "404 Not Found" in line:
                    print("Punkt montowania nie istnieje.")
                    return False
                elif "200 OK" in line:
                    print("Połączenie OK.")
                    return True
            return False

        except Exception as e:
            print(f"Błąd podczas przetwarzania nagłówków: {e}")
            return False

    def process_data(self):
        try:
            while True:
                gga_data = self.get_gga_bytes()
                if gga_data:
                    self.socket.sendall(gga_data)
                    data = self.socket.recv(self.buffer_size)
                    if not data:
                        break
                    self.stream.write(data)
                    raw_data, parsed_data = self.nmr.read()

                    if raw_data and b"GNGGA" in raw_data:
                        quality, hdop = parsed_data.quality, parsed_data.HDOP
                        if b"GNRMC" in raw_data:
                            self.display_gps_data(parsed_data, quality, hdop)
                            self.append_to_file(parsed_data, quality, hdop)
                else:
                    print("Nie otrzymano danych GGA.")
                    time.sleep(0.1)

        except (socket.timeout, socket.error) as e:
            print(f"Błąd połączenia podczas odbierania danych: {e}")
        except Exception as e:
            print(f"Inny błąd podczas przetwarzania danych: {e}")

    def display_gps_data(self, parsed_data, quality, hdop):
        time_str, lat, lon = parsed_data.time, parsed_data.lat, parsed_data.lon
        speed_kmh = parsed_data.spd * 1.852
        print(f"Time: {time_str}, Latitude: {lat:.10f}, Longitude: {lon:.10f}, Speed_kmh: {speed_kmh:.5f}, Quality: {quality}, HDOP: {hdop:.3f}")

    def append_to_file(self, parsed_data, quality, hdop):
        if not self.save_to_file:
            return

        time_str, lat, lon = parsed_data.time, parsed_data.lat, parsed_data.lon
        speed_kmh = parsed_data.spd * 1.852

        try:
            with open(self.file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(["Time", "Latitude", "Longitude", "Speed", "Quality", "HDOP"])
                writer.writerow([time_str, lat, lon, speed_kmh, quality, hdop])
        except Exception as e:
            print(f"Błąd podczas zapisu do pliku: {e}")

    def cleanup_connection(self):
        print('Zamykanie połączenia...')
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError as e:
                print(f"Błąd podczas zamykania socketu: {e}")
            finally:
                self.socket.close()
        if self.stream:
            self.stream.close()
        print('Połączenie zamknięte.')

if __name__ == '__main__':
    config = {
        'user': 'pwmgr/adamwrb:Globus7142001',
        'caster': 'system.asgeupos.pl',
        'port': 8080,
        'mountpoint': '/RTN4G_VRS_RTCM32',
        'verbose': True,
        'save_to_file': True,
        'file_path': '/home/pi/mss/lc29h_gps_rtk_hat_code/python/rtk_rover/test6.csv',
        'serial_port': '/dev/ttyUSB0',
        'serial_baudrate': 115200
    }
    client = NtripClient(**config)
    client.read_data()