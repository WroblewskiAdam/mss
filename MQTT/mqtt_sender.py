import paho.mqtt.client as mqtt
import random
import time

# Konfiguracja MQTT
MQTT_BROKER = "mss-mqttbroker.ddns.net"  # Zmień na swój dynamiczny DNS
MQTT_PORT = 1883
MQTT_TOPIC = "test/random_number"  # Temat testowy
MQTT_USER = "gpsuser"  # Ustaw nazwę użytkownika MQTT
MQTT_PASSWORD = "########"  # Ustaw hasło MQTT

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

try:
    while True:
        random_number = random.randint(1, 100)
        message = str(random_number) # Konwertuj liczbę na string przed wysłaniem
        client.publish(MQTT_TOPIC, message)
        print(f"Wysłano: {message} na temat: {MQTT_TOPIC}")
        time.sleep(2) # Wysyłaj co 2 sekundy

except KeyboardInterrupt:
    print("Zatrzymano nadajnik testowy")
finally:
    client.loop_stop()
    client.disconnect()