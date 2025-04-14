import pygame
import cv2
import numpy as np
from djitellopy import Tello
import time
import threading
import math

# --- Configuration ---
CONTROL_SENSITIVITY = 75  # Lower values make it less sensitive (range 0-100)
AXIS_DEADZONE = 0.15      # Ignore small stick movements around the center
TRIGGER_THRESHOLD = 0.1   # How much the trigger needs to be pressed to be considered "on"

# --- Controller Mapping (Adjust if necessary) ---
# Axes
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5 # Usually axis 5 for RT on Xbox controllers

# Buttons (Optional - can add more commands later)
# A_BUTTON = 0
# B_BUTTON = 1
# X_BUTTON = 2
# Y_BUTTON = 3

# --- Global Variables ---
tello = None
keep_running = True
is_flying = False
video_frame = None
frame_lock = threading.Lock()
joystick = None
rt_pressed = False # Safety trigger status

# --- Helper Functions ---
def initialize_joystick():
    """Initializes Pygame and finds the first joystick."""
    global joystick
    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("Error: No joystick found.")
        return False
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick initialized: {joystick.get_name()}")
        return True

def scale_axis(value):
    """Scales joystick axis value (-1 to 1) to Tello command value (-100 to 100) with deadzone."""
    if abs(value) < AXIS_DEADZONE:
        return 0
    # Scale the value outside the deadzone
    if value > 0:
        scaled_value = (value - AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    else:
        scaled_value = (value + AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    return int(scaled_value * CONTROL_SENSITIVITY)

def video_stream_handler():
    """Continuously fetches video frames from Tello in a separate thread."""
    global video_frame, keep_running
    print("Video stream handler started.")
    frame_read = tello.get_frame_read()
    while keep_running:
        try:
            current_frame = frame_read.frame
            if current_frame is not None:
                with frame_lock:
                    # Make a copy to avoid race conditions if OpenCV modifies it
                    video_frame = current_frame.copy()
            else:
                # If we get None frame for a bit, maybe connection is dropping
                time.sleep(0.01) # Avoid busy-waiting
        except Exception as e:
            print(f"Error getting video frame: {e}")
            time.sleep(0.1) # Wait a bit longer if there's an error
    print("Video stream handler stopping.")

def display_telemetry(frame):
    """Gets telemetry data and draws it on the frame."""
    if frame is None:
        return None # Nothing to draw on

    display_frame = frame.copy() # Work on a copy

    # Get Telemetry (handle potential errors/missing data)
    try:
        height = tello.get_height() # Barometer height (cm)
        tof_distance = tello.get_distance_tof() # Time-of-Flight distance (cm) - better for low altitude
        speed_x = tello.get_speed_x()
        speed_y = tello.get_speed_y()
        speed_z = tello.get_speed_z()
        # Tello SDK doesn't directly provide heading from North. Yaw is relative.
        yaw = tello.get_yaw() # Degrees relative to start/last reset
        battery = tello.get_battery()
        flight_time = tello.get_flight_time()

        # Calculate horizontal speed
        horizontal_speed = math.sqrt(speed_x**2 + speed_y**2)

        status = "OK" if is_flying else "Landed" # Simple status
        if battery < 20: status = "LOW BATT"

        # Prepare text lines
        text_lines = [
            f"Status: {status} {' (RT Active)' if rt_pressed else ''}",
            f"Altitude (ToF): {tof_distance} cm",
            # f"Altitude (Baro): {height} cm", # Uncomment if you prefer barometer
            f"Speed (H): {horizontal_speed:.1f} cm/s",
            f"Speed (V): {speed_z:.1f} cm/s",
            f"Yaw: {yaw} deg",
            f"Battery: {battery}%",
            f"Flight Time: {flight_time}s",
        ]

        # Draw text on the frame
        y_pos = 30
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0) # Green
        thickness = 1
        line_type = cv2.LINE_AA

        for line in text_lines:
            cv2.putText(display_frame, line, (10, y_pos), font, font_scale, color, thickness, line_type)
            y_pos += 25 # Move down for the next line

        return display_frame

    except Exception as e:
        print(f"Error getting telemetry: {e}")
        # Draw a simple error message if telemetry fails
        cv2.putText(display_frame, "Telemetry Error", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return display_frame


# --- Main Control Loop ---
def run_tello_control():
    global keep_running, is_flying, tello, video_frame, rt_pressed

    # --- Initialize Tello ---
    tello = Tello()
    try:
        tello.connect()
        print("Tello connected.")
        print(f"Battery: {tello.get_battery()}%")
        tello.streamon()
        print("Video stream on.")
        # Set initial speed to moderate (default is 100, can be too fast)
        tello.set_speed(50)

    except Exception as e:
        print(f"Failed to connect to Tello or start stream: {e}")
        keep_running = False
        return

    # --- Start Video Thread ---
    video_thread = threading.Thread(target=video_stream_handler, daemon=True)
    video_thread.start()
    time.sleep(1) # Give video thread a moment to start

    # --- Main Loop ---
    try:
        while keep_running:
            # --- Handle Pygame Events (Controller Input) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    keep_running = False
                    break
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        keep_running = False
                        break
                    # Optional: Add keyboard overrides if needed
                    # if event.key == pygame.K_t:
                    #     if not is_flying:
                    #         print("Taking off (Keyboard override)")
                    #         tello.takeoff()
                    #         is_flying = True
                    # if event.key == pygame.K_l:
                    #      if is_flying:
                    #         print("Landing (Keyboard override)")
                    #         tello.land()
                    #         is_flying = False
                elif event.type == pygame.JOYAXISMOTION:
                    # Check if the right trigger is pressed
                    if event.axis == RIGHT_TRIGGER:
                         # Axis value ranges from -1 (released) to 1 (fully pressed)
                        rt_value = joystick.get_axis(RIGHT_TRIGGER)
                        rt_pressed = rt_value > (TRIGGER_THRESHOLD * 2 - 1) # Scale threshold to axis range
                        # print(f"RT Value: {rt_value:.2f}, Pressed: {rt_pressed}") # Debugging

                # Optional: Add button handling here if needed
                # elif event.type == pygame.JOYBUTTONDOWN:
                #     if event.button == A_BUTTON: # Example: Takeoff with A
                #         if not is_flying and rt_pressed: # Only takeoff if RT is held
                #              print("Takeoff requested (A button)")
                #              tello.takeoff()
                #              is_flying = True

            if not keep_running: break # Exit if ESC or Quit event happened

            # --- Get Controller State ---
            left_x = scale_axis(joystick.get_axis(LEFT_STICK_X))
            # Invert Y axis (pygame up is -1, tello up is positive)
            left_y = -scale_axis(joystick.get_axis(LEFT_STICK_Y))
            right_x = scale_axis(joystick.get_axis(RIGHT_STICK_X))
             # Invert Y axis (pygame up is -1, tello forward is positive)
            right_y = -scale_axis(joystick.get_axis(RIGHT_STICK_Y))

            # --- Tello Control Logic ---
            # left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity
            rc_command = [0, 0, 0, 0]

            if rt_pressed:
                # --- Takeoff logic (if not flying) ---
                if not is_flying:
                     # Basic takeoff: Just pressing RT takes off.
                     # You might want to add a button press (like 'A') required as well.
                    print("Right Trigger pressed - Taking off!")
                    try:
                        tello.takeoff()
                        is_flying = True
                        time.sleep(1) # Short pause after takeoff
                    except Exception as e:
                        print(f"Takeoff failed: {e}")
                        rt_pressed = False # Prevent trying again immediately if failed

                # --- Flying logic (if flying) ---
                if is_flying:
                    # Map sticks to RC commands
                    rc_command[0] = left_x  # Left/Right Yaw (Rotation)
                    rc_command[1] = right_y # Forward/Backward
                    rc_command[2] = left_y  # Up/Down
                    rc_command[3] = right_x # Left/Right Strafe (if desired, otherwise set to 0)
                    # *** Current mapping based on your request:
                    # Left Stick Up/Down -> ud (rc_command[2])
                    # Left Stick Left/Right -> yaw (rc_command[0])
                    # Right Stick Up/Down -> fb (rc_command[1])
                    # Right Stick Left/Right -> lr (rc_command[3]) - Set to 0 if you don't want strafing with right stick
                    # --> Let's adjust rc_command based on the initial request description:
                    # Left Stick U/D = Up/Down (left_y -> rc_command[2])
                    # Left Stick L/R = Yaw Left/Right (left_x -> rc_command[3]) <-- THIS IS YAW
                    # Right Stick U/D = Forward/Backward (right_y -> rc_command[1])
                    # Right Stick L/R = --- UNUSED --- (set rc_command[0] to 0)
                    # Right Trigger = Safety

                    # Corrected mapping based on description:
                    lr = 0 # No sideways movement from sticks in description
                    fb = right_y # Forward/Backward from Right Stick Y
                    ud = left_y # Up/Down from Left Stick Y
                    yaw = left_x # Turn Left/Right from Left Stick X

                    tello.send_rc_control(lr, fb, ud, yaw)

            else: # Right Trigger Released
                if is_flying:
                    print("Right Trigger released - Landing!")
                    try:
                        tello.send_rc_control(0, 0, 0, 0) # Stop all movement first
                        tello.land()
                        is_flying = False
                    except Exception as e:
                        print(f"Landing failed: {e}")
                        # Might still be flying, state could be inconsistent
                else:
                     # Ensure drone stays put if on the ground and RT released
                     tello.send_rc_control(0, 0, 0, 0)


            # --- Display Video and Telemetry ---
            local_frame = None
            with frame_lock:
                if video_frame is not None:
                    local_frame = video_frame.copy() # Work with a copy

            if local_frame is not None:
                # Add telemetry overlay
                display_frame = display_telemetry(local_frame)
                cv2.imshow("Tello Control", display_frame)
            else:
                # Show a placeholder if no frame is available yet
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Connecting to video...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tello Control", placeholder)


            # Check for exit key (e.g., 'q') within OpenCV window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                keep_running = False
                break

            time.sleep(0.02) # Small delay to prevent CPU hogging

    except Exception as e:
        print(f"An error occurred in the main loop: {e}")
    finally:
        # --- Cleanup ---
        print("Cleaning up...")
        keep_running = False # Signal video thread to stop

        if is_flying:
            print("Ensuring Tello lands...")
            try:
                tello.send_rc_control(0,0,0,0) # Stop movement before landing
                tello.land()
            except Exception as e:
                print(f"Error during final landing: {e}")

        if tello:
             try:
                 tello.streamoff()
             except Exception as e:
                 print(f"Error stopping stream: {e}")
             try:
                 tello.end() # Disconnects
             except Exception as e:
                 print(f"Error during Tello end: {e}")

        video_thread.join(timeout=2) # Wait for video thread to finish
        cv2.destroyAllWindows()
        pygame.quit()
        print("Cleanup complete. Exiting.")

# --- Start the Application ---
if __name__ == "__main__":
    if initialize_joystick():
        run_tello_control()
    else:
        print("Exiting due to joystick initialization failure.")