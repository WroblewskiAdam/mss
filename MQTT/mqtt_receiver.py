import paho.mqtt.client as mqtt
import time

# Konfiguracja MQTT
MQTT_BROKER = "mss-mqttbroker.ddns.net"  # Zmień na swój dynamiczny DNS
MQTT_PORT = 1883
MQTT_TOPIC = "test/random_number"  # Temat testowy
MQTT_USER = "gpsuser"  # Ustaw nazwę użytkownika MQTT
MQTT_PASSWORD = "Globus7142001@"  # Ustaw hasło MQTT

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Odbiornik MQTT połączony z serwerem testowym")
        client.subscribe(MQTT_TOPIC) # Subskrybuj temat po połączeniu
    else:
        print(f"Błąd połączenia z serwerem MQTT, kod: {rc}")

last_receive_time = None # Inicjalizacja czasu ostatniej odebranej wiadomości

def on_message(client, userdata, msg):
    global last_receive_time # Użyj zmiennej globalnej
    current_time = time.time()
    random_number_received = msg.payload.decode() # Odbierz wiadomość i dekoduj z bajtów na string

    if last_receive_time is not None: # Sprawdź, czy to nie pierwsza wiadomość
        time_elapsed = current_time - last_receive_time
        print(f"Odebrano wiadomość na temat: {msg.topic}, liczba: {random_number_received}, czas od poprzedniej wiadomości: {time_elapsed:.6f} sekundy")
    else:
        print(f"Odebrano pierwszą wiadomość na temat: {msg.topic}, liczba: {random_number_received}")

    last_receive_time = current_time # Zaktualizuj czas ostatniej odebranej wiadomości

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(MQTT_USER, MQTT_PASSWORD) # Ustaw uwierzytelnianie
client.connect(MQTT_BROKER, MQTT_PORT, 60)

client.loop_forever() # Pętla nieskończona, czekaj na wiadomości