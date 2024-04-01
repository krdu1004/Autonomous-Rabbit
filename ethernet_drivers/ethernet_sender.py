import socket
import struct

# IP and port for the computer
HOST = '169.254.222.165'  # MÅ ENDRES TIL ZED BOX
PORT = 49152

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the host and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen(1)

print('Waiting for connection...')

# Accept a connection
client_socket, addr = server_socket.accept()
print('Connected by', addr)

while True:
    # Boolean values
    redBool = bool(input("Red LED: "))
    greenBool = bool(input("Green LED: "))
    brakeBool = bool(input("Brake: "))

    # Floating-point values
    angle = float(input("Angle: "))
    speed = float(input("Speed: "))

    # Pack data into byte array
    data = struct.pack('<3?2f', redBool, greenBool, brakeBool, angle, speed)

    # Send data
    client_socket.sendall(data)

    # Receive data from the client
    #data = client_socket.recv(1024)
    #if not data:
    #    break
    #print('Received:', data.decode())
    #print('Received:', struct.unpack('f', data))
    #print('Received:', data)
# Close the connection
client_socket.close()
server_socket.close()