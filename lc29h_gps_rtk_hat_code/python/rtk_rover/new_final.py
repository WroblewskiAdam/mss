import serial
import socket
import base64
import threading
import time
from pynmeagps import NMEAReader

class NTRIPClient:
    GGA_STRING = "$GNGGA,115713.000,3149.301528,N,11706.920684,E,1,17,0.88,98.7,M,-3.6,M,,*58\r\n"
    
    def __init__(self, serial_port, baudrate, caster, port, mountpoint, user):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.sock = None
        self.serial_com = None
        self.NEMAreader = None
    
    def connect_ntrip(self):
        """Łączy się z serwerem NTRIP i pobiera dane korekcyjne"""
        ntrip_request = f"GET {self.mountpoint} HTTP/1.0\r\n" \
                        f"User-Agent: NTRIP PythonClient\r\n" \
                        f"Authorization: Basic {base64.b64encode(self.user.encode()).decode()}\r\n\r\n"
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.caster, self.port))
            self.sock.send(ntrip_request.encode())
            response = self.sock.recv(1024)
            if b"200 OK" in response:
                print("Connected to NTRIP caster")
                print(response.decode())
                return True
            else:
                print("Failed to connect to NTRIP caster")
                print(response.decode())
                return False
        except Exception as e:
            print(f"Error connecting to NTRIP caster: {e}")
    
    def send_GGA_to_NTRIP(self):
        if self.sock:
            self.sock.send(self.GGA_STRING.encode())
            while True:
                response = self.sock.recv(4096)
                print("Received RTCM data from server: %d bytes." % len(response))
                hex_string = ' '.join(format(b, '02x') for b in response).upper()
                time.sleep(1)
            # print(hex_string)
    
    def connect_gnss(self):
        """Connects to GPS RTK module via UART"""
        try:
            self.serial_com = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.NEMAreader = NMEAReader(self.serial_com)
            print("Connected to GNSS module")
        except Exception as e:
            print(f"Error connecting to GNSS module: {e}")
    

    def read_and_send_gga(self):
        """Odczytuje dane GGA z GNSS i wysyła je do serwera NTRIP"""
        if not self.serial_com:
            print("GNSS module not connected.")
            return
        
        while True:
            line = self.serial_com.readline().decode(errors='ignore').strip()
            if line.startswith("$GNGGA"):
                print(f"Sending GGA: {line}")
                self.send_GGA_to_NTRIP(line)
            time.sleep(1)  # Wysyłanie co 1 sekundę
    

    def run(self):
        """Uruchamia klienta NTRIP"""
        self.connect_ntrip()
        for i in range(30):
            print(i+1)
            self.send_GGA_to_NTRIP()
            time.sleep(1)

        # self.connect_ntrip()
        # self.connect_gnss()
        # self.read_and_send_gga()
        
if __name__ == "__main__":
    client = NTRIPClient(
        serial_port='/dev/ttyUSB0', 
        baudrate=115200, 
        caster='system.asgeupos.pl', 
        port=8080, 
        mountpoint='/RTN4G_VRS_RTCM32', 
        user='pwmgr/adamwrb:Globus7142001'
    )
    client.run()
