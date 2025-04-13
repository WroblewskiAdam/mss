import RPi.GPIO as GPIO
import time

# Lista numerów pinów GPIO do odczytania (użyj numeracji BCM)
gpio_pins = [16, 25, 24, 23]

# Ustaw tryb numeracji pinów na BCM
GPIO.setmode(GPIO.BCM)

# Ustaw wszystkie piny jako wejścia z rezystorami pull-down
for pin in gpio_pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        pin_states = {}
        for pin in gpio_pins:
            input_state = GPIO.input(pin)
            if input_state == GPIO.HIGH:
                pin_states[pin] = "HIGH"
            else:
                pin_states[pin] = "LOW"

        # Wyświetl stan pinów w terminalu
        print(f"Stan pinów: {pin_states}")

        time.sleep(0.01)  # Odczekaj pół sekundy

except KeyboardInterrupt:
    print("Program zakończony.")
finally:
    GPIO.cleanup()