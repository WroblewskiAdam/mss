import serial
import socket
import base64
import time
import csv
from pynmeagps import NMEAReader
import os
from datetime import datetime

class GPSrtk:
    def __init__(self, serial_port, baudrate, caster, port, mountpoint, user, save_file = False, filename = None):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.sock = None
        self.serial_com = None
        self.nmr = None

        self.GGAdata = None
        self.VTGdata = None

        self.base_path = '/home/pi/mss/data/'
        self.filename = 'pi4.csv' if filename is None else filename
        self.save_file = save_file

    def connect_ntrip(self):
        """Connect to ntrip server"""
        ntrip_request = (
            f"GET {self.mountpoint} HTTP/1.0\r\n"
            f"User-Agent: NTRIP PythonClient\r\n"
            f"Authorization: Basic {base64.b64encode(self.user.encode()).decode()}\r\n\r\n"
        )
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.caster, self.port))
            self.sock.send(ntrip_request.encode())
            response = self.sock.recv(1024)
            if b"200 OK" in response:
                print("Connected to NTRIP caster")
                return True
            else:
                print("Failed to connect to NTRIP caster")
                return False
        except Exception as e:
            print(f"Error connecting to NTRIP caster: {e}")
            return False
    
    def connect_gnss(self):
        """Connect to GPS module via UART"""
        try:
            self.serial_com = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.nmr = NMEAReader(self.serial_com)
            print("Connected to GNSS module")
            return True
        except Exception as e:
            print(f"Error connecting to GNSS module: {e}")
            return False

    def send_gga_to_ntrip(self, gga_string):
        """Send GGA message to NTRIP server, returns RTCM message with corrections as response"""
        if self.sock:
            try:
                gga_string += "\r\n"
                self.sock.send(gga_string.encode())
                # print(f"Sent GGA: {gga_string}")
                response = self.sock.recv(4096)
                return response
            except Exception as e:
                print(f"Error sending GGA to NTRIP: {e}")
                self.reconnect_ntrip()

    def receive_rtcm(self):
        """Odbiera dane RTCM z serwera NTRIP"""
        try:
            response = self.sock.recv(4096)
            if response:
                print(f"Received {len(response)} bytes of RTCM data.")
            else:
                print("Lost connection to NTRIP server, reconnecting...")
                self.reconnect_ntrip()
        except Exception as e:
            print(f"Error receiving RTCM data: {e}")
            self.reconnect_ntrip()
    
    def reconnect_ntrip(self):
        """Ponawia połączenie z serwerem NTRIP"""
        print("Reconnecting to NTRIP server...")
        if self.sock:
            self.sock.close()
        time.sleep(2)
        self.connect_ntrip()

    def append_to_file(self):
        if self.save_file:
            try:
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
                        writer.writerow(["Time", "Latitude", "Longitude", "Speed", "Quality"])
                    writer.writerow([self.GGAdata.time, f"{self.GGAdata.lat:.8f}", f"{self.GGAdata.lon:.8f}", self.VTGdata.sogk, self.GGAdata.quality])
            except Exception as e:
                print(f"Błąd przy zapisie do pliku: {e}")

    def print_data(self):
        if self.GGAdata and self.VTGdata:
            print(f"Time: {self.GGAdata.time}, Lat: {self.GGAdata.lat:.8f}, "
                f"Lon: {self.GGAdata.lon:.8f}, Speed: {self.VTGdata.sogk} km/s, "
                f"Fix Quality: {self.GGAdata.quality}")


    def run(self):
        # Establish NTRIP connaction
        if not self.connect_ntrip():
            return
        
        # Establish connection with GPS module
        if not self.connect_gnss():
            return

        while True:
            try:
                if self.serial_com:
                    raw_data, parsed_data = self.nmr.read()
                    if b"GNGGA" in raw_data:
                        RTCM_response = self.send_gga_to_ntrip(raw_data.decode())
                        self.serial_com.write(RTCM_response)
                        self.GGAdata = parsed_data
                    if b"GNVTG" in raw_data:
                        self.VTGdata = parsed_data
                        self.print_data()
                        self.append_to_file()
            
            except KeyboardInterrupt:
                print("Stopping client...")
                if self.sock:
                    self.sock.close()
                if self.serial_com:
                    self.serial_com.close()
                break
            except Exception as e:
                # print(f"Unexpected error: {e}")
                pass
                # time.sleep(2)

if __name__ == "__main__":
    client = GPSrtk(
        serial_port='/dev/ttyUSB0', 
        baudrate=115200, 
        caster='system.asgeupos.pl', 
        port=8080, 
        mountpoint='/RTN4G_VRS_RTCM32', 
        user='pwmgr/adamwrb:Globus7142001',
        save_file=True,
    )
    client.run()