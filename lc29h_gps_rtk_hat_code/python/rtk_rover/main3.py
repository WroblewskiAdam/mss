import socket
import sys
import datetime
import base64
import time
import csv
import serial
from pynmeagps import NMEAReader

version = 0.2
useragent = f"NTRIP JCMBsoftPythonClient/{version:.1f}"

factor = 2
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 1

class NtripClient:
    def __init__(self, buffer=50, user="", port=2101, caster="", mountpoint="", verbose=False, maxConnectTime=0, save_to_file=False, file_path="data.csv"):
        self.buffer = buffer
        self.user = base64.b64encode(user.encode()).decode("utf-8")
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.verbose = verbose
        self.maxConnectTime = maxConnectTime
        self.save_to_file = save_to_file
        self.file_path = file_path
        self.socket = None
        self.stream = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)

    def get_mountpoint_bytes(self):
        request = f"GET {self.mountpoint} HTTP/1.1\r\nUser-Agent: {useragent}\r\nAuthorization: Basic {self.user}\r\n\r\n"
        if self.verbose:
            print(request)
        return request.encode('ascii')

    def get_gga_bytes(self):
        while True:
            raw_data, parsed_data = self.nmr.read()
            if b"GNGGA" in raw_data:
                return raw_data
    
    
    def read_data(self):
        reconnectTry, sleepTime = 1, 1
        while reconnectTry <= maxReconnect:
            if self.verbose:
                sys.stderr.write(f'Connection {reconnectTry} of {maxReconnect}\n')
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.socket.connect_ex((self.caster, self.port)):
                self.handle_reconnect(reconnectTry, sleepTime)
                reconnectTry += 1
                continue
            
            self.socket.settimeout(10)
            self.socket.sendall(self.get_mountpoint_bytes())
            if not self.process_headers():
                return
            
            self.socket.sendall(self.get_gga_bytes())
            self.process_data()
            self.cleanup_connection()
            return
    
    def handle_reconnect(self, reconnectTry, sleepTime):
        if reconnectTry < maxReconnect:
            sys.stderr.write(f"{datetime.datetime.now()} No Connection to NtripCaster. Trying again in {sleepTime} seconds\n")
            time.sleep(sleepTime)
            sleepTime = min(sleepTime * factor, maxReconnectTime)
    
    def process_headers(self):
        while True:
            response = self.socket.recv(4096).decode('utf-8').split("\r\n")
            for line in response:
                if not line:
                    return True
                if "SOURCETABLE" in line:
                    sys.stderr.write("Mount point does not exist\n")
                    sys.exit(1)
                elif "401 Unauthorized" in line:
                    sys.stderr.write("Unauthorized request\n")
                    sys.exit(1)
                elif "404 Not Found" in line:
                    sys.stderr.write("Mount Point does not exist\n")
                    sys.exit(2)
                elif "200 OK" in line:
                    return True
        return False
    
    def process_data(self):
        quality, hdop = -1, -1
        while True:
            try:
                data = self.socket.recv(self.buffer)
                if not data:
                    break
                self.stream.write(data)
                raw_data, parsed_data = self.nmr.read()
                if b"GNGGA" in raw_data:
                    quality, hdop = parsed_data.quality, parsed_data.HDOP
                if b"GNRMC" in raw_data:
                    self.display_gps_data(parsed_data, quality, hdop)
                    self.append_to_file(parsed_data, quality, hdop)
            except (socket.timeout, socket.error) as e:
                if self.verbose:
                    sys.stderr.write(f'Connection Error: {e}\n')
                break
    
    def display_gps_data(self, parsed_data, quality, hdop):
        time, lat, lon = parsed_data.time, parsed_data.lat, parsed_data.lon
        speed_kmh = parsed_data.spd * 1.852
        print(f"Time: {time}, Latitude: {lat:.10f}, Longitude: {lon:.10f}, Speed_kmh: {speed_kmh:.5f}, Quality: {quality}, HDOP: {hdop:.3f}")

    def append_to_file(self, parsed_data, quality, hdop):
        if not self.save_to_file:
            return
        time, lat, lon = parsed_data.time, parsed_data.lat, parsed_data.lon
        speed_kmh = parsed_data.spd * 1.852
        try:
            with open(self.file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(["Time", "Latitude", "Longitude", "Speed", "Quality", "HDOP"])
                writer.writerow([time, lat, lon, speed_kmh, quality, hdop])
        except Exception as e:
            print(f"Błąd przy zapisie do pliku: {e}")
    
    def cleanup_connection(self):
        if self.verbose:
            sys.stderr.write('Closing Connection\n')
        if self.socket:
            self.socket.close()
        self.stream.close()

if __name__ == '__main__':
    config = {
        'user': 'pwmgr/adamwrb:Globus7142001',
        'caster': 'system.asgeupos.pl',
        'port': 8080,
        'mountpoint': '/RTN4G_VRS_RTCM32',
        'verbose': True,
        'save_to_file': True,
        'file_path': '/home/pi/mss/lc29h_gps_rtk_hat_code/python/rtk_rover/test6.csv'
    }
    client = NtripClient(**config)
    client.read_data()
