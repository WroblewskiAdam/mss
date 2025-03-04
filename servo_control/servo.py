import time
from adafruit_servokit import ServoKit

# Inicjalizacja sterownika PCA9685
# Domyślny adres I2C to 0x40
kit = ServoKit(channels=16)  # 16 kanałów w PCA9685

# Wybierz numer kanału serwa (0-15)
SERVO_CHANNEL = 0

# Możesz dostosować parametry serwa jeśli potrzeba
kit.servo[SERVO_CHANNEL].set_pulse_width_range(500, 2500)  # Standardowe wartości dla większości serw

def przeskaluj_predkosc(predkosc_uzytkownika):
    """
    Przeskalowuje prędkość z zakresu 1-10 na 1-100
    """
    return int(1 + ((predkosc_uzytkownika - 1) * (99/9)))

def plynny_ruch(kanal, kat_koncowy, predkosc_uzytkownika):
    """
    Wykonuje płynny ruch serwa do zadanego kąta z zadaną prędkością
    predkosc_uzytkownika: 1-10, gdzie:
    1 = najwolniej
    10 = maksymalna prędkość (bez opóźnień)
    """
    # Bezpieczne pobranie kąta początkowego
    try:
        aktualna_pozycja = kit.servo[kanal].angle
        kat_poczatkowy = int(aktualna_pozycja) if aktualna_pozycja is not None else 0
    except:
        kat_poczatkowy = 0
    
    # Przeskalowanie prędkości na zakres wewnętrzny
    predkosc = przeskaluj_predkosc(predkosc_uzytkownika)
    
    # Dla prędkości 100 (użytkownik podał 10) wykonujemy bezpośredni ruch
    if predkosc >= 100:
        kit.servo[kanal].angle = kat_koncowy
        return
        
    # Dla pozostałych prędkości obliczamy opóźnienie
    opoznienie = 0.1 * ((100 - predkosc) / 100) ** 2
    
    # Określenie kierunku ruchu
    krok = 1 if kat_koncowy > kat_poczatkowy else -1
    
    for kat in range(kat_poczatkowy, kat_koncowy + krok, krok):
        kit.servo[kanal].angle = kat
        time.sleep(opoznienie)

def pobierz_wartosc(prompt, min_val, max_val):
    """
    Pobiera wartość od użytkownika z określonym zakresem
    """
    while True:
        try:
            wartosc = int(input(prompt))
            if min_val <= wartosc <= max_val:
                return wartosc
            print(f"Wartość musi być między {min_val} a {max_val}")
        except ValueError:
            print("Proszę wprowadzić poprawną liczbę całkowitą")

def pobierz_aktualna_pozycje(kanal):
    """
    Bezpiecznie pobiera aktualną pozycję serwa
    Zwraca 0 jeśli pozycja nie może być odczytana
    """
    try:
        pozycja = kit.servo[kanal].angle
        return int(pozycja) if pozycja is not None else 0
    except:
        return 0

try:
    print("Program sterowania serwem - wpisz 'q' aby zakończyć")
    print("Prędkość: 1-10")
    print("1 = najwolniej")
    print("10 = maksymalna prędkość (ruch natychmiastowy)")
    
    while True:
        print("\nAktualna pozycja serwa:", pobierz_aktualna_pozycje(SERVO_CHANNEL), "°")
        
        # Pobieranie danych od użytkownika
        wybor = input("\nWprowadź kąt (0-180) lub 'q' aby zakończyć: ")
        
        if wybor.lower() == 'q':
            break
            
        try:
            kat = int(wybor)
            if not (0 <= kat <= 180):
                print("Kąt musi być w zakresie 0-180°")
                continue
                
            predkosc = pobierz_wartosc("Wprowadź prędkość (1-10, gdzie 1=najwolniej, 10=najszybciej): ", 1, 10)
            
            print(f"\nWykonuję ruch do {kat}° z prędkością {predkosc}")
            plynny_ruch(SERVO_CHANNEL, kat, predkosc)
            
        except ValueError:
            print("Nieprawidłowa wartość. Spróbuj ponownie.")

except KeyboardInterrupt:
    print("\nProgram zatrzymany przez użytkownika")

finally:
    # Zatrzymanie w aktualnej pozycji
    print("\nKoniec programu")
