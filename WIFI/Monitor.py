import socket
import struct
import pandas as pd
import datetime

NUM_ADC_CHANNELS = 5
BUF_SIZE = 64
SAMPLE_INTERVAL_MS = 10
LEN_TIMESTAMP = 2
sizeof_float = 4
sizeof_uint32 = 4

counter = 0

# Server Config
# SERVER_IP = "192.168.0.114" # TP-Link_AA24
SERVER_IP = "10.49.22.33" # RedRover
SERVER_PORT = 1234 

# TCP Server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((SERVER_IP, SERVER_PORT))
server_socket.listen(1)

print(f"Listening on {SERVER_IP}:{SERVER_PORT}...")

client_socket, client_address = server_socket.accept()
print(f"Connected to {client_address}")

now = datetime.datetime.now()
filename = now.strftime("%Y%m%d_%H%M%S")
filename += ".csv"

with open(filename, mode='w', encoding='utf-8', newline='') as file:
    while True:
        try:
            counter += 1
            data = client_socket.recv(sizeof_uint32 + LEN_TIMESTAMP * sizeof_uint32 + BUF_SIZE * NUM_ADC_CHANNELS * sizeof_float)
            if counter == 1: continue
            elif counter >= 2: counter = 2
            if not data:
                print("Client disconnected.")
                break

            # float_count = len(data) // sizeof_float
            # float_array = [struct.unpack('<f', data[i * sizeof_float:(i + 1) * sizeof_float])[0] for i in range(float_count)]            
            print(f"Received {len(data)} bytes")
            if len(data) != sizeof_uint32 + LEN_TIMESTAMP * sizeof_uint32 + BUF_SIZE * NUM_ADC_CHANNELS * sizeof_float:
                print("Data length mismatch: ", len(data))
                print(data)
                continue

            Package_count = int.from_bytes(data[:sizeof_uint32], byteorder='little')

            timestamp_s = int.from_bytes(data[sizeof_uint32:LEN_TIMESTAMP * sizeof_uint32], byteorder='little')
            timestamp_us = int.from_bytes(data[LEN_TIMESTAMP * sizeof_uint32:(LEN_TIMESTAMP+1) * sizeof_uint32], byteorder='little')
            time_stamp = timestamp_s + timestamp_us / 1000000

            output_array = [struct.unpack('<f', data[i * sizeof_float:(i + 1) * sizeof_float])[0] for i in range(1 + LEN_TIMESTAMP, 1 + LEN_TIMESTAMP + NUM_ADC_CHANNELS * BUF_SIZE)]
            assert len(output_array) == NUM_ADC_CHANNELS * BUF_SIZE, "ADC length mismatch"

            time_stamp_array = [time_stamp + i * SAMPLE_INTERVAL_MS / 1000 for i in range(len(output_array) // NUM_ADC_CHANNELS)]

            # Create a DataFrame
            dataframe_raw = [[Package_count, time_stamp_array[i], output_array[NUM_ADC_CHANNELS*i], output_array[NUM_ADC_CHANNELS*i+1], output_array[NUM_ADC_CHANNELS*i+2], output_array[NUM_ADC_CHANNELS*i+3], output_array[NUM_ADC_CHANNELS*i+4]] 
                              for i in range(len(output_array) // NUM_ADC_CHANNELS)]
            dataframe = pd.DataFrame(dataframe_raw, columns=['Package Count', 'Time Stamp', 'ADC1', 'ADC2', 'ADC3', 'ADC4', 'ADC5']) # A2-34, A3-39, A4-36, A7-32, A9-33
            # dataframe['Time Stamp'] = dataframe['Time Stamp'].apply(lambda x: '{:.6f}'.format(x))
            dataframe = dataframe.round(6)
            dataframe.to_csv(file, header=False, index=False)

        except Exception as e:
            print(f"Error: {e}")
            break

client_socket.close()
server_socket.close()
print("Server shutdown.")
