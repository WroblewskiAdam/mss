import socket
import sys
import datetime
import base64
import time
import csv
import serial
from pynmeagps import NMEAReader

class NtripClient:
    def __init__(self, user, caster, port, mountpoint, verbose=False, save_to_file=False, file_path="data.csv"):
        self.user = base64.b64encode(user.encode()).decode("utf-8")
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.verbose = verbose
        self.save_to_file = save_to_file
        self.file_path = file_path
        self.socket = None
        self.stream = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)

    def get_mountpoint_bytes(self):
        request = (
            f"GET {self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.caster}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP Client\r\n"
            f"Authorization: Basic {self.user}\r\n"
            f"Connection: close\r\n\r\n"
        )
        if self.verbose:
            print("\n### Wysłane zapytanie do NTRIP ###\n", request)
        return request.encode('ascii')

    def get_gga_bytes(self):
        """ Pobiera GGA lub wysyła domyślne """
        while True:
            raw_data, parsed_data = self.nmr.read()
            if b"GNGGA" in raw_data:
                return raw_data
        return b"$GNGGA,120000.00,5200.0000,N,02100.0000,E,4,12,0.8,100.0,M,50.0,M,,*68\r\n"
    
    def connect(self):
        """ Obsługuje połączenie NTRIP i przetwarza dane """
        try:
            self.socket = socket.create_connection((self.caster, self.port), timeout=10)
            self.socket.sendall(self.get_mountpoint_bytes())
            response = self.socket.recv(1024).decode()
            if self.verbose:
                print("\n### Odpowiedź serwera NTRIP ###\n", response)
            if "200 OK" not in response:
                sys.stderr.write("Błąd połączenia NTRIP - serwer odrzucił żądanie.\n")
                return
            self.socket.sendall(self.get_gga_bytes())
            self.process_data()
        except socket.error as e:
            sys.stderr.write(f"Błąd połączenia: {e}\n")
            if "Connection reset by peer" in str(e):
                time.sleep(5)
                self.connect()
        finally:
            if self.socket:
                self.socket.close()
            self.stream.close()

    def process_data(self):
        """ Odczytuje i przetwarza dane GPS """
        while True:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                self.stream.write(data)
                self.stream.flush()
                raw_data, parsed_data = self.nmr.read()
                if b"GNGGA" in raw_data:
                    quality = getattr(parsed_data, 'quality', None)
                    time = getattr(parsed_data, 'time', None)
                    print(f"Quality: {quality}, time: {time}")
                    if quality == 4:
                        print("RTK FIXED!")
                if b"GNRMC" in raw_data:
                    self.save_gps_data(parsed_data)
            except Exception as e:
                sys.stderr.write(f"Błąd odczytu: {e}\n")
                break

    def save_gps_data(self, parsed_data):
        if not self.save_to_file:
            return
        try:
            with open(self.file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([parsed_data.time, parsed_data.lat, parsed_data.lon, getattr(parsed_data, 'quality', 'N/A')])
        except Exception as e:
            print(f"Błąd zapisu: {e}")

if __name__ == '__main__':
    config = {
        'user': 'pwmgr/adamwrb:Globus7142001',
        'caster': 'system.asgeupos.pl',
        'port': 8080,
        'mountpoint': '/RTN4G_VRS_RTCM32',
        'verbose': True,
        'save_to_file': True,
        'file_path': '/home/pi/mss/lc29h_gps_rtk_hat_code/python/rtk_rover/test5.csv'
    }
    client = NtripClient(**config)
    client.connect()