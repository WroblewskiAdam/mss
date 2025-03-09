import serial
import socket
import base64
import threading

def connect_ntrip(caster, port, mountpoint, user):
    """Łączy się z serwerem NTRIP i pobiera dane korekcyjne"""
    ntrip_request = f"GET {mountpoint} HTTP/1.0\r\n" \
                    f"User-Agent: NTRIP PythonClient\r\n" \
                    f"Authorization: Basic {base64.b64encode(user.encode()).decode()}\r\n\r\n"
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((caster, port))
    sock.send(ntrip_request.encode())
    return sock

def read_ntrip_data(sock, serial_port):
    """Czyta dane z serwera NTRIP i przesyła je do modułu GNSS"""
    while True:
        data = sock.recv(1024)
        if not data:
            break
        serial_port.write(data)

def read_gnss_data(serial_port, sock):
    """Odczytuje dane NMEA z modułu GNSS, wysyła GGA do NTRIP i wyświetla je"""
    while True:
        line = serial_port.readline().decode(errors='ignore').strip()
        if line:
            print("GNSS:", line)
            if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                try:
                    sock.sendall(line.encode() + b"\r\n")
                    print("Wysłano GGA do NTRIP")
                except Exception as e:
                    print("Błąd wysyłania GGA:", e)

def main():
    # Konfiguracja
    serial_port = '/dev/ttyUSB0'
    baudrate = 115200
    caster = 'system.asgeupos.pl'
    port = 8080
    mountpoint = '/RTN4G_VRS_RTCM32'
    user = 'pwmgr/adamwrb:Globus7142001'
    
    # Połączenie z modułem GNSS
    ser = serial.Serial(serial_port, baudrate, timeout=1)
    print("Połączono z modułem GNSS")
    
    # Połączenie z NTRIP
    sock = connect_ntrip(caster, port, mountpoint, user)
    print("Połączono z serwerem NTRIP")
    
    # Wątek do odbioru danych NTRIP
    ntrip_thread = threading.Thread(target=read_ntrip_data, args=(sock, ser), daemon=True)
    ntrip_thread.start()
    
    # Odczyt danych GNSS i wysyłanie GGA
    read_gnss_data(ser, sock)
    
if __name__ == "__main__":
    main()
