# pi_sender.py
import socket
import time
import random  # replace with actual DHT sensor readings

HOST = '10.210.211.57'  # laptop IP where ROS2 bridge runs
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        sock.connect((HOST, PORT))
        break
    except:
        print("Waiting for bridge to be ready...")
        time.sleep(2)

try:
    while True:
        # Replace with actual DHT readings
        temp = 21.8 + random.random()
        hum = 50 + random.random()
        msg = f"{temp},{hum}"
        sock.sendall(msg.encode())
        print("Sent:", msg)
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting")
finally:
    sock.close()
