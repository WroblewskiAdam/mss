import paho.mqtt.client as mqtt
import random
import time

# Konfiguracja MQTT
MQTT_BROKER = "mss-mqttbroker.ddns.net"  # Zmień na swój dynamiczny DNS
MQTT_PORT = 1883
MQTT_TOPIC = "test/random_number"  # Temat testowy
MQTT_USER = "gpsuser"  # Ustaw nazwę użytkownika MQTT
MQTT_PASSWORD = "Globus7142001@"  # Ustaw hasło MQTT

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Nadajnik MQTT połączony z serwerem testowym")
    else:
        print(f"Błąd połączenia z serwerem MQTT, kod: {rc}")

client = mqtt.Client()
client.on_connect = on_connect
client.username_pw_set(MQTT_USER, MQTT_PASSWORD) # Ustaw uwierzytelnianie
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()


last_send_time = time.time()
try:
    while True:
        for i in range(10):
            current_time = time.time()
            time_elapsed = current_time - last_send_time

            message = str(i) # Konwertuj liczbę na string przed wysłaniem
            client.publish(MQTT_TOPIC, message)
            last_send_time = current_time # Zaktualizuj czas ostatniego wysłania

            print(f"Wysłano: {message} na temat: {MQTT_TOPIC}, czas od poprzedniej wiadomości: {time_elapsed:.6f} sekundy")
            time.sleep(0.1) # Wysyłaj co 2 sekundy

except KeyboardInterrupt:
    print("Zatrzymano nadajnik testowy")
finally:
    client.loop_stop()
    client.disconnect()