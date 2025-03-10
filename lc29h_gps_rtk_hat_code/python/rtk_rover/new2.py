import serial
import socket
import base64
import threading

def parse_nmea(sentence):
    """Parsuje wiadomości NMEA i wyciąga wymagane dane"""
    parts = sentence.split(',')
    
    if sentence.startswith("$GPGGA") or sentence.startswith("$GNGGA"):
        if len(parts) > 6:
            time_utc = parts[1]
            lat = parts[2]
            lat_dir = parts[3]
            lon = parts[4]
            lon_dir = parts[5]
            quality = parts[6]

            if time_utc and lat and lon:
                lat = convert_to_decimal(lat, lat_dir)
                lon = convert_to_decimal(lon, lon_dir)
                print(f"Czas UTC: {format_time(time_utc)}, Lat: {lat}, Lon: {lon}, Jakość GPS: {quality}")

    elif sentence.startswith("$GPRMC") or sentence.startswith("$GNRMC"):
        if len(parts) > 7:
            speed_knots = parts[7]
            try:
                speed_kmh = float(speed_knots) * 1.852  # Przeliczenie z węzłów na km/h
                print(f"Prędkość: {speed_kmh:.2f} km/h")
            except ValueError:
                pass
        append_to_file(time_utc, lat, lon, speed_kmh, quality)
    

def append_to_file(time, lat, lon, speed, quality):
    try:
        with open('/home/pi/mss/lc29h_gps_rtk_hat_code/python/rtk_rover/test6.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            if file.tell() == 0:
                writer.writerow(["Time", "Latitude", "Longitude", "Speed", "Quality"])
            writer.writerow([time, lat, lon, speed, quality])
    except Exception as e:
        print(f"Błąd podczas zapisu do pliku: {e}")


def convert_to_decimal(coord, direction):
    """Konwertuje współrzędne NMEA na format dziesiętny"""
    if not coord:
        return None
    degrees = int(float(coord) / 100)
    minutes = float(coord) % 100
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def format_time(time_utc):
    """Formatuje czas UTC z NMEA"""
    if len(time_utc) >= 6:
        return f"{time_utc[:2]}:{time_utc[2:4]}:{time_utc[4:6]}"
    return time_utc

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
    """Odczytuje dane NMEA z modułu GNSS, wysyła GGA do NTRIP i wyświetla tylko wybrane informacje"""
    while True:
        line = serial_port.readline().decode(errors='ignore').strip()
        if line:
            if line.startswith("$GPGGA") or line.startswith("$GNGGA") or line.startswith("$GPRMC") or line.startswith("$GNRMC"):
                parse_nmea(line)

            if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                try:
                    sock.sendall(line.encode() + b"\r\n")
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
