import RPi.GPIO as GPIO
import time

# Ustawienie numeracji pinów
GPIO.setmode(GPIO.BCM)

# Definicja pinów
PINS = [16, 23, 24, 25]
#16 dół 
# 23 góra


# Konfiguracja pinów jako wyjścia i ustawienie w stan niski
for pin in PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

try:
    # Ustawienie pinu 16 w stan wysoki
    GPIO.output(16, GPIO.HIGH)
    time.sleep(0.1)  # Czekaj 0.5 sekundy
    
    # Ustawienie pinu 16 w stan niski
    GPIO.output(16, GPIO.LOW)
finally:
    # Czyszczenie ustawień GPIO
    GPIO.cleanup()
