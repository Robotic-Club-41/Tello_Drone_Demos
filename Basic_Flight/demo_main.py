
# -------- Start of Setup -------- #

import socket
import cv2

tello_ip = '192.168.10.1'
tello_port = 8889
command_port = 8889
video_port = 11111

# UDP socket
sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind to all interfaces on the local port
sock.bind(('', command_port))

# Function to send a command and print the response

def send_command(command):
  try:
    # Send command
    sock.sendto(command.encode(), (tello_ip, tello_port))
    print(f"Sent: {command}")
    # Wait for response 
    response, ip = sock.recvfrom(1024)
    print("Received:", response.decode())
  except Exception as e:
    print("Error:", e)


# -------- End of Setup -------- #

# Debug marker
print("Marker")

# Enter SDK mode
send_command("command")

# Takeoff command
send_command("takeoff")

# Ascend to 200 cm
send_command("up 200")

# Backflip
send_command("flip f")

# Frontflip
send_command("flip b")

# Backflip
send_command("flip f")

# Frontflip
send_command("flip b")

# Landing command
send_command("land")

# Close the socket
sock.close()