from pynmeagps import NMEAReader
import socket
import serial
import base64

class NtripClient:
    def __init__(self):
        # Dane logowania do castera
        self.user = base64.b64encode(bytes("pwmgr/adamwrb:Globus7142001", 'utf-8')).decode("utf-8")
        self.port = 8086
        self.caster = "system.asgeupos.pl"
        self.mountpoint = "/CBKA_RTCM_3_2"
        self.verbose = True

        # Inicjalizacja NMEA i portu szeregowego
        self.buffer = 1024
        self.stream = serial.Serial('/dev/ttyS0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)
        self.socket = None

    def getMountPointBytes(self):
        # Tworzenie żądania GET dla mountpoint
        mount_point_request = f"GET {self.mountpoint} HTTP/1.1\r\n" \
                              f"User-Agent: NTRIP PythonClient\r\n" \
                              f"Authorization: Basic {self.user}\r\n\r\n"
        return bytes(mount_point_request, 'ascii')

    def parse_gga(self, parsed_data):
        # Parsowanie pozycji i czasu z wiadomości GGA
        if parsed_data.msgID == 'GGA':
            latitude = parsed_data.lat
            longitude = parsed_data.lon
            timestamp = parsed_data.time
            return latitude, longitude, timestamp
        return None, None, None

    def parse_vtg(self, parsed_data):
        # Parsowanie prędkości z wiadomości VTG
        if parsed_data.msgID == 'VTG':
            speed = parsed_data.sogk
            return speed
        return None

    def connect_and_read(self):
        try:
            # Połączenie z casterem
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.caster, self.port))
            self.socket.sendall(self.getMountPointBytes())
            print("Połączono z casterem. Oczekiwanie na dane...")

            while True:
                data = self.stream.read(self.buffer)
                if data:
                    raw, parsed = self.nmr.read()
                    if parsed:
                        print(parsed)
                        latitude, longitude, timestamp = self.parse_gga(parsed)
                        speed = self.parse_vtg(parsed)

                        if latitude and longitude and timestamp:
                            print(f"Pozycja: {latitude}, {longitude}, Czas: {timestamp}")
                        if speed:
                            print(f"Prędkość: {speed} km/h")

        except Exception as e:
            print(f"Błąd: {e}")
        finally:
            if self.socket:
                self.socket.close()
            self.stream.close()

if __name__ == "__main__":
    client = NtripClient()
    client.connect_and_read()
