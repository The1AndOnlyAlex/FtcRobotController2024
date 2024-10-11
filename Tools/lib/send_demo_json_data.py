import json
import socket
import time
import threading
import random

# Global variable to keep track of the client connection status
client_connected = False
client_socket = None
server_socket = None
out = None

# Function to start the server and handle client connections
def start_server():
    global client_connected, client_socket, out, server_socket

    try:
        # Open a server socket on the local machine
        HOST = '127.0.0.1'  # Listen on localhost
        PORT = 12345  # Same port as the client

        # Create the server socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"Server listening on {HOST}:{PORT}...")

        # Wait for the PC client to connect
        client_socket, addr = server_socket.accept()
        client_connected = True
        print(f"PC connected from {addr}")

        # Set up the output stream to send data to the client
        out = client_socket.makefile('w')

        # Handle input from the client
        handle_client_input(client_socket)

    except Exception as e:
        print(f"Server Error: {e}")

# Function to handle receiving JSON data from the client
def handle_client_input(client_socket):
    try:
        with client_socket.makefile('r') as inp:
            while True:
                client_data = inp.readline().strip()
                if client_data:
                    # Parse the received JSON data from the client
                    client_json = json.loads(client_data)
                    print(f"Received input from client: {client_json}")
    except Exception as e:
        print(f"Error receiving client input: {e}")

# Function to send demo JSON data to the client
def send_demo_data():
    global client_connected, out, client_socket

    while True:
        if client_connected and out is not None:
            try:
                # Simulate telemetry data (e.g., motor power, battery voltage, and time)
                current_time = time.time()
                motor_power = random.uniform(0, 1)  # Simulate motor power between 0 and 1
                battery_voltage = random.uniform(11.0, 13.0)  # Simulate battery voltage between 11V and 13V

                # Create a JSON object with the telemetry data
                json_data = {
                    "time": current_time,
                    "motorPower": motor_power,
                    "batteryVoltage": battery_voltage
                }

                # Send the JSON data to the connected client
                out.write(json.dumps(json_data) + "\n")
                out.flush()  # Ensure the data is sent immediately

                # Print the sent data for debugging
                print(f"Sent data: {json_data}")

            except (BrokenPipeError, ConnectionResetError):
                # Handle client disconnection
                print("Client disconnected")
                client_connected = False
                client_socket.close()
                break  # Break out of the loop if the client disconnects

        # Sleep for a short period before sending the next data point
        time.sleep(1)

if __name__ == "__main__":
    # Start the server in a separate thread
    server_thread = threading.Thread(target=start_server)
    server_thread.start()

    # Continuously send demo data while the server is running
    send_demo_data()
