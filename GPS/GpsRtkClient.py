import serial
import socket
import base64
import time
import csv
import os
from datetime import datetime

class GPSrtk:
    def __init__(self, serial_port, baudrate, caster, port, mountpoint, user, filename, save_file = True):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.sock = None
        self.serial_com = None

        self.GGAdata = None
        self.VTGdata = None

        self.base_path = '/home/pi/mss/data/'
        self.filename = filename
        self.save_file = save_file
        self.hostname = socket.gethostname()
        self.fix = None
        self.last_gga_sent_to_ntrip = None
        self.init_gga_sent = False
        self.send_gga_interval = 10
        self.message_dt = 0


    def connect_ntrip(self):
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
        try:
            self.serial_com = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print("Connected to GNSS module")
            return True
        except Exception as e:
            print(f"Error connecting to GNSS module: {e}")
            return False

    def send_gga_to_ntrip(self, gga_string):
        if self.sock:
            try:
                gga_string += "\r\n"
                self.sock.send(gga_string.encode())
                response = self.sock.recv(4096)
                return response
            except Exception as e:
                print(f"Error sending GGA to NTRIP: {e}")
                self.reconnect_ntrip()

    def receive_rtcm(self):
        try:
            response = self.sock.recv(4096)
            if response:
                return response
            else:
                print("Lost connection to NTRIP server, reconnecting...")
                self.reconnect_ntrip()
        except Exception as e:
            print(f"Error receiving RTCM data: {e}")
            self.reconnect_ntrip()

    def reconnect_ntrip(self):
        print("Reconnecting to NTRIP server...")
        if self.sock:
            self.sock.close()
        time.sleep(2)
        self.connect_ntrip()

    def append_to_file(self):
        if self.save_file and self.GGAdata and self.VTGdata:
            try:
                date_str = datetime.now().strftime("%d-%m-%y")
                dir = f"data/{date_str}/pi41/"
                os.makedirs(dir, exist_ok=True)
                file_path = os.path.join(dir, self.filename)
                local_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                with open(file_path, 'a', newline='') as file:
                    writer = csv.writer(file)
                    if file.tell() == 0:
                        writer.writerow(["dt, Local Time", "GPS Time", "Latitude", "Longitude", "Altitude", "Heading", "Speed", "Quality"])
                    writer.writerow([
                        f"{self.message_dt:.3f}", local_time, self.GGAdata.time,
                        f"{self.GGAdata.lat:.8f}", f"{self.GGAdata.lon:.8f}", f"{self.GGAdata.altitude:.2f}", 
                        f"{self.VTGdata.cogt:.2f}", self.VTGdata.sogk,
                        self.GGAdata.quality
                    ])
            except Exception as e:
                print(f"Błąd przy zapisie do pliku: {e}")


    def print_data(self):
        if self.GGAdata and self.VTGdata:
            local_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"dt: {self.message_dt:.3f},T: {local_time}, Tgps: {self.GGAdata.time}, Lat: {self.GGAdata.lat:.4f}, "
                  f"Lon: {self.GGAdata.lon:.4f}, H: {self.GGAdata.altitude:.2f}, C: {self.VTGdata.cogt:.2f}, "
                  f"V: {self.VTGdata.sogk:.3f} km/s, Q: {self.GGAdata.quality}")


    def process_gga(self, parsed_data, raw_data):
        self.fix = parsed_data.quality
        self.GGAdata = parsed_data

        if self.fix != 0:
            current_time = time.time()
            if current_time - self.last_gga_sent_to_ntrip >= self.send_gga_interval or not self.init_gga_sent:
                RTCM_response = self.send_gga_to_ntrip(raw_data)
                print(f"Sent GGA to NTRIP")
                self.last_gga_sent_to_ntrip = time.time()
                self.init_gga_sent = True
            else:
                RTCM_response = self.receive_rtcm()
                if RTCM_response:
                    self.serial_com.write(RTCM_response)
        else:
            print("Waiting for GNSS fix...")

    def process_vtg(self, parsed_data):
        if self.fix != 0:
            current_time = time.time()
            self.message_dt = current_time - self.last_gga_time_local
            self.last_gga_time_local = current_time

            self.VTGdata = parsed_data
            self.print_data()
            self.append_to_file()
        else:
            print("Waiting for GNSS fix...")

    def parse_gga(self, line):
        try:
            parts = line.split(',')
            if len(parts) < 15:
                return None
            lat = self.nmea_to_decimal(parts[2], parts[3])
            lon = self.nmea_to_decimal(parts[4], parts[5])
            quality = int(parts[6]) if parts[6].isdigit() else 0
            altitude = float(parts[9]) if parts[9] else 0.0  # Dodajemy wysokość
            return type("GGA", (), {
                "time": parts[1],
                "lat": lat,
                "lon": lon,
                "quality": quality,
                "altitude": altitude
            })()
        except Exception as e:
            print(f"Error parsing GGA: {e}")
            return None

    def parse_vtg(self, line):
        try:
            parts = line.split(',')
            if len(parts) < 9:
                return None
            cogt = float(parts[1]) if parts[1] else 0.0
            sogk = float(parts[7]) if parts[7] else 0.0
            return type("VTG", (), {
                "cogt": cogt,
                "sogk": sogk
            })()
        except Exception as e:
            print(f"Error parsing VTG: {e}")
            return None

    def nmea_to_decimal(self, coord, direction):
        if not coord or not direction:
            return 0.0
        degrees = int(coord[:2 if direction in ['N', 'S'] else 3])
        minutes = float(coord[2 if direction in ['N', 'S'] else 3:])
        decimal = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def run(self, stop_event=None):
        if not self.connect_ntrip():
            return

        if not self.connect_gnss():
            return

        self.last_gga_sent_to_ntrip = time.time()
        self.last_gga_time_local = time.time()

        while True:
            if stop_event and stop_event.is_set():
                print("[GPS] Otrzymano sygnał zatrzymania.")
                break

            try:
                if not self.serial_com:
                    continue

                line = self.serial_com.readline().decode(errors='ignore').strip()
                if not line.startswith('$'):
                    continue

                if "GGA" in line:
                    parsed = self.parse_gga(line)
                    if parsed:
                        self.process_gga(parsed, line)

                elif "VTG" in line:
                    parsed = self.parse_vtg(line)
                    if parsed:
                        self.process_vtg(parsed)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[GPS] Błąd w pętli: {e}")
                continue

        print("[GPS] Zamykanie połączeń...")
        if self.sock:
            self.sock.close()
        if self.serial_com:
            self.serial_com.close()


if __name__ == "__main__":
    client = GPSrtk(
        serial_port='/dev/ttyUSB0',
        baudrate=115200,
        caster='system.asgeupos.pl',
        port=8080,
        mountpoint='/RTN4G_VRS_RTCM32',
        user='pwmgr/adamwrb:Globus7142001',
        filename='dupa.csv',
        save_file=True,
    )
    client.run()
