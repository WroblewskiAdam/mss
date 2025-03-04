import RPi.GPIO as GPIO
import time

# Konfiguracja numeracji pinów
GPIO.setmode(GPIO.BCM)

pin = 17
# Ustawienie pinu 22 jako wyjście
GPIO.setup(pin, GPIO.OUT)

try:
    while True:
        GPIO.output(pin, GPIO.HIGH)  # Włączenie pinu
        time.sleep(0.5)  # Pauza 0.5 sekundy
        GPIO.output(pin, GPIO.LOW)   # Wyłączenie pinu
        time.sleep(0.5)  # Pauza 0.5 sekundy
except KeyboardInterrupt:
    print("Zatrzymano program.")
    GPIO.cleanup()  # Resetowanie ustawień GPIO