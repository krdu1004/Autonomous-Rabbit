import socket

# Set the IP address and port of the microcontroller
microcontroller_ip = "192.168.1.2"  # Replace with the actual IP address of your microcontroller
microcontroller_port = 12345  # Replace with the desired port number

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the microcontroller
client_socket.connect((microcontroller_ip, microcontroller_port))

# Send data to the microcontroller
data_to_send = "Hello from the computer!"
client_socket.sendall(data_to_send.encode())

# Receive data from the microcontroller
received_data = client_socket.recv(1024).decode()
print(f"Received from microcontroller: {received_data}")

# Close the socket
client_socket.close()