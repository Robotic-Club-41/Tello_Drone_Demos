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


# Step 1: Enter SDK mode
send_command("command")

# Step 2: Takeoff command
send_command("takeoff")

#send_command("streamon")

# Capture the video using OpenCV
"""
cap = cv2.VideoCapture("udp://0.0.0.0:111111", cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("Error: Could not open video stream.")
else:
    print("Video stream opened. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to receive frame.")
            break
        cv2.imshow('Tello Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
""" 

# Ascend to 110 cm
send_command("up 110")

# Rotate 360 degrees clockwise

send_command("cw 360")

# Backflip
send_command("flip b")


# Step 3: Landing command
send_command("land")

# Close the socket
sock.close()
