from lib_nrf24 import NRF24
import time
import spidev
import RPi.GPIO as GPIO

# Konfiguracja GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Konfiguracja NRF24 - dokładnie jak w przykładzie
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 25)
time.sleep(1)
radio.setRetries(15,15)
radio.setPayloadSize(32)
radio.setChannel(0x60)
radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.startListening()
radio.printDetails()

print("Rozpoczęto nasłuchiwanie...")

try:
    while True:
        if radio.available():
            received_message = []
            radio.read(received_message, radio.getDynamicPayloadSize())
            
            # Konwertujemy otrzymane bajty na znaki
            message = ''.join([chr(x) for x in received_message if x >= 32 and x <= 126])
            print(f"Otrzymano: {message}")
            print(f"Surowe dane: {received_message}")
            
            # Przygotowanie odpowiedzi ACK
            ack_payload = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0]
            radio.writeAckPayload(1, ack_payload)
                
        time.sleep(0.01)
            
except KeyboardInterrupt:
    print("\nZakończono program")
    GPIO.cleanup()
