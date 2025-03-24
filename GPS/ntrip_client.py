import socket
import base64
import time

# NTRIP Caster information
NTRIP_IP = "system.asgeupos.pl"
NTRIP_PORT = 8080
NTRIP_USERNAME = "pwmgr"
NTRIP_PASSWORD = "Globus7142001"
NTRIP_MOUNT_POINT = "RTN4G_VRS_RTCM32"

GGA_STRING = "$GNGGA,115713.000,3149.301528,N,11706.920684,E,1,17,0.88,98.7,M,-3.6,M,,*58\r\n"

# Create a TCP socket and connect to the NTRIP Caster
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((NTRIP_IP, NTRIP_PORT))
    print("Connected.")
except socket.error as err:
    print("Connect fail,exit.")
    exit(1)

# Send login request
Authorization = (NTRIP_USERNAME + ":" + NTRIP_PASSWORD)
Authorization = base64.b64encode(Authorization.encode()).decode()
requestHead = f"""GET /{NTRIP_MOUNT_POINT} HTTP/1.0\r\nUser-Agent: Quectel GNSS\r\nHost: {NTRIP_IP}\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic {Authorization}\r\n\r\n"""
sock.send(requestHead.encode())
print("send auth msg to NTRIP caster:%s" % requestHead)

# Receive and parse the NTRIP login response
response = sock.recv(1024)
print("receive from server:%s" % response.decode())
if "ICY 200 OK" not in response.decode():
    print("login to NTRIP caster fail,exit.")
    exit(1)
print('login to NTRIP caster successful.')

# Send GGA data to NtripCaster
while True:
    sock.send(GGA_STRING.encode())
    print("\r\n\r\nSend GGA to NTRIP caster:%s" % GGA_STRING)
    response = sock.recv(4096)
    print("Receive RTCM data from server:%d bytes.(should send the RTCM data to GNSS RTK module)" % len(response))
    hex_string = ' '.join(format(b, '02x') for b in response).upper()
    print(hex_string)
    time.sleep(1)
