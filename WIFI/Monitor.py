import socket
import struct

NUM_ADC_CHANNELS = 5
BUF_SIZE = 64
sizeof_float = 4
sizeof_uint32 = 4

# Server Config
SERVER_IP = "192.168.0.114"
SERVER_PORT = 1234 

# TCP Server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((SERVER_IP, SERVER_PORT))
server_socket.listen(1)

print(f"Listening on {SERVER_IP}:{SERVER_PORT}...")

client_socket, client_address = server_socket.accept()
print(f"Connected to {client_address}")

with open("sensor_data.raw", "wb") as file:
    while True:
        try:
            data = client_socket.recv(7 * sizeof_uint32 + BUF_SIZE * NUM_ADC_CHANNELS * sizeof_float)
            if not data:
                print("Client disconnected.")
                break

            # float_count = len(data) // sizeof_float
            # float_array = [struct.unpack('<f', data[i * sizeof_float:(i + 1) * sizeof_float])[0] for i in range(float_count)]            
            print(f"Received {len(data)} bytes")

            file.write(data)
            file.flush()

        except Exception as e:
            print(f"Error: {e}")
            break

client_socket.close()
server_socket.close()
print("Server shutdown.")
