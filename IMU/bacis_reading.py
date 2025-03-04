import time
import board
import busio
import adafruit_adxl34x

# Inicjalizacja I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Inicjalizacja akcelerometru ADXL345
try:
    accelerometer = adafruit_adxl34x.ADXL345(i2c)

    # Ustawienie częstotliwości odczytu na 200 Hz
    accelerometer.data_rate = adafruit_adxl34x.DataRate.RATE_200_HZ

    # Ustawienie zakresu pomiarowego (opcjonalne, ale może poprawić dokładność)
    accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G

except ValueError as e:
    print(f"Błąd inicjalizacji ADXL345: {e}")
    print("Upewnij się, że ADXL345 jest poprawnie podłączony i adres I2C jest prawidłowy.")
    exit()

# Częstotliwości
sampling_frequency = 200  # Hz - Częstotliwość odczytu z akcelerometru
output_frequency = 10   # Hz - Docelowa częstotliwość wyjściowa
decimation_factor = int(sampling_frequency / output_frequency)  # Współczynnik decymacji

# Parametry filtra średniej kroczącej
window_size = 10  # Wielkość okna filtra - ile próbek bierzemy do średniej

# Inicjalizacja list historii dla filtra
acceleration_history_x = []
acceleration_history_y = []
acceleration_history_z = []

# Licznik próbek
sample_counter = 0

def moving_average_filter(history, new_value, window_size):
    history.append(new_value)
    if len(history) > window_size:
        history.pop(0)  # Usuń najstarszą wartość, jeśli okno jest pełne
    return sum(history) / len(history) if history else 0

try:
    while True:
        # Odczyt surowych danych przyspieszenia
        raw_x, raw_y, raw_z = accelerometer.acceleration

        # Filtrowanie danych
        filtered_x = moving_average_filter(acceleration_history_x, raw_x, window_size)
        filtered_y = moving_average_filter(acceleration_history_y, raw_y, window_size)
        filtered_z = moving_average_filter(acceleration_history_z, raw_z, window_size)

        # Decymacja (Downsampling)
        if sample_counter % decimation_factor == 0:
            # Wyświetlanie danych (tylko co 'decimation_factor' próbek)
            print(f"Surowe: X={raw_x:.2f} Y={raw_y:.2f} Z={raw_z:.2f} m/s^2 \t Filtrowane: X={filtered_x:.2f} Y={filtered_y:.2f} Z={filtered_z:.2f} m/s^2")

        # Inkrementacja licznika próbek
        sample_counter += 1

        # Opóźnienie
        time.sleep(1.0 / sampling_frequency)  # Opóźnienie dostosowane do częstotliwości próbkowania

except KeyboardInterrupt:
    print("Program zakończony przez użytkownika.")
except Exception as e:
    print(f"Wystąpił błąd: {e}")
finally:
    print("Koniec programu.")