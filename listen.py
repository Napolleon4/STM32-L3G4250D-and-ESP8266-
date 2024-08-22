import socket
import struct

UDP_IP = "192.168.1.101"
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on \n IP:{UDP_IP}\nPORT:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    if len(data) == 2:  # Check if the received data is 2 bytes long (int16_t)
        value = struct.unpack('<h', data)[0]  # '<h' format code is for little-endian int16_t
        print(f"Received message: {value}")
    else:
        print(f"Received data of incorrect length: {len(data)} bytes")
