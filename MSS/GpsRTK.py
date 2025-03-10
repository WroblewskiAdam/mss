import serial
import socket
import base64
import time

class GPSrtk:
    def __init__(self, serial_port, baudrate, caster, port, mountpoint, user):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.caster = caster
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.sock = None
        self.serial_com = None
    
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
                    
                    NMEAmessage = self.serial_com.readline().decode(errors='ignore').strip()
                    if NMEAmessage.startswith("$GNGGA"):
                        print(NMEAmessage)
                        RTCM_response = self.send_gga_to_ntrip(NMEAmessage)
                        self.serial_com.write(RTCM_response)
                    if NMEAmessage.startswith("$GNVTG"):
                        print(NMEAmessage)
                        print()
                        
            except KeyboardInterrupt:
                print("Stopping client...")
                if self.sock:
                    self.sock.close()
                if self.serial_com:
                    self.serial_com.close()
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                time.sleep(2)

if __name__ == "__main__":
    client = GPSrtk(
        serial_port='/dev/ttyUSB0', 
        baudrate=115200, 
        caster='system.asgeupos.pl', 
        port=8080, 
        mountpoint='/RTN4G_VRS_RTCM32', 
        user='pwmgr/adamwrb:Globus7142001'
    )
    client.run()
