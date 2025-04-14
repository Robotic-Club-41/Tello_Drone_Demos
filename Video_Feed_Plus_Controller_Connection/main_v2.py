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
TRIGGER_THRESHOLD = 0.1   # How much the trigger needs to be pressed (-1 to 1 range)

# --- Controller Mapping (Adjust if necessary for your controller/OS) ---
# Axes
LEFT_STICK_X = 0  # Left (-1) / Right (+1)
LEFT_STICK_Y = 1  # Up (-1) / Down (+1)  -> Often inverted by Pygame
RIGHT_STICK_X = 3 # Left (-1) / Right (+1)
RIGHT_STICK_Y = 4 # Up (-1) / Down (+1)  -> Often inverted by Pygame
RIGHT_TRIGGER = 5 # Released (-1) / Fully Pressed (+1)

# Buttons (Optional - currently unused, but can be added)
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
    """Initializes Pygame and finds the first connected joystick."""
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
        # Print number of axes/buttons to help with debugging mapping
        print(f"  Axes: {joystick.get_numaxes()}")
        print(f"  Buttons: {joystick.get_numbuttons()}")
        return True

def scale_axis(value):
    """Scales joystick axis value (-1 to 1) to Tello command value (-100 to 100)
       applying deadzone and sensitivity."""
    if abs(value) < AXIS_DEADZONE:
        return 0
    # Scale the value outside the deadzone to the full range
    if value > 0:
        scaled_value = (value - AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    else:
        scaled_value = (value + AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    return int(scaled_value * CONTROL_SENSITIVITY)

def video_stream_handler():
    """Continuously fetches video frames from Tello in a separate thread."""
    global video_frame, keep_running
    print("Video stream handler started.")
    frame_read = None
    try:
        frame_read = tello.get_frame_read()
    except Exception as e:
        print(f"Error getting frame reader: {e}")
        keep_running = False # Stop main loop if we can't get video reader
        return

    while keep_running:
        try:
            if frame_read:
                current_frame = frame_read.frame
                if current_frame is not None:
                    with frame_lock:
                        # Make a copy to avoid race conditions if OpenCV modifies it later
                        video_frame = current_frame.copy()
                else:
                    # If we get None frame, wait a bit
                    time.sleep(0.01)
            else:
                print("Error: Frame reader not available.")
                time.sleep(0.5) # Wait longer if reader is missing
        except Exception as e:
            print(f"Error in video stream handler: {e}")
            time.sleep(0.1) # Wait a bit longer if there's an error

    print("Video stream handler stopping.")

def display_telemetry(frame):
    """Gets telemetry data and draws it onto the provided frame."""
    if frame is None:
        return None # Nothing to draw on

    display_frame = frame.copy() # Work on a copy

    # Get Telemetry (handle potential errors/missing data)
    try:
        # height = tello.get_height() # Barometer height (cm) - Alternative
        tof_distance = tello.get_distance_tof() # Time-of-Flight distance (cm) - Better low altitude
        speed_x = tello.get_speed_x()
        speed_y = tello.get_speed_y()
        speed_z = tello.get_speed_z()
        # Tello SDK yaw is relative to start/last reset, not absolute North.
        yaw = tello.get_yaw() # Degrees relative
        battery = tello.get_battery()
        flight_time = tello.get_flight_time()

        # Calculate horizontal speed magnitude
        horizontal_speed = math.sqrt(speed_x**2 + speed_y**2)

        # Determine flight status string
        status = "Landed"
        if is_flying:
            status = "Flying"
        if rt_pressed and is_flying:
             status += " (Armed)"
        elif not rt_pressed and is_flying:
             status += " (Landing...)"
        elif rt_pressed and not is_flying:
             status += " (Takeoff Pending)"

        if battery < 20: status += " LOW BATT!"

        # Prepare text lines for display
        text_lines = [
            f"Status: {status}",
            f"Altitude (ToF): {tof_distance} cm",
            f"Speed H: {horizontal_speed:.1f} cm/s | V: {speed_z:.1f} cm/s",
            f"Yaw (Rel): {yaw} deg",
            f"Battery: {battery}%",
            f"Flight Time: {flight_time}s",
        ]

        # Draw text onto the frame
        y_pos = 30
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0) # Green text
        shadow_color = (0, 0, 0) # Black shadow/outline
        thickness = 1
        shadow_thickness = 2
        line_type = cv2.LINE_AA

        for line in text_lines:
            # Draw shadow first
            cv2.putText(display_frame, line, (11, y_pos + 1), font, font_scale, shadow_color, shadow_thickness, line_type)
            # Draw text on top
            cv2.putText(display_frame, line, (10, y_pos), font, font_scale, color, thickness, line_type)
            y_pos += 25 # Move down for the next line

        return display_frame

    except Exception as e:
        print(f"Telemetry Error: {e}")
        # Draw a simple error message on the frame if telemetry fails
        cv2.putText(display_frame, "Telemetry Error", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        return display_frame


# --- Main Control Function ---
def run_tello_control():
    global keep_running, is_flying, tello, video_frame, rt_pressed

    # --- Initialize Tello ---
    tello = Tello()
    try:
        print("Connecting to Tello...")
        tello.connect()
        print("Tello connected.")
        print(f"Battery: {tello.get_battery()}%")

        # Query firmware version (good check)
        # print(f"Firmware: {tello.query_firmware_version()}")

        # Reset any leftover RC commands
        tello.send_rc_control(0, 0, 0, 0)

        print("Starting video stream...")
        tello.streamon()
        print("Video stream started.")
        # Set initial speed (default is 100, can be too fast for beginners)
        tello.set_speed(60)
        print("Speed set to 60.")

    except Exception as e:
        print(f"Error during Tello initialization: {e}")
        keep_running = False
        # Clean up pygame if joystick was initialized
        if joystick:
            pygame.quit()
        return

    # --- Start Video Handler Thread ---
    video_thread = threading.Thread(target=video_stream_handler, daemon=True)
    video_thread.start()
    # Give the video thread a moment to start and get the first frame
    print("Waiting for first video frame...")
    time.sleep(2.0)

    # --- Main Loop ---
    print("\n--- Ready to Fly ---")
    print("Hold Right Trigger (RT) to take off and fly.")
    print("Release Right Trigger (RT) to land.")
    print("Press 'q' in the video window to quit.")

    last_rc_command_time = time.time()

    try:
        while keep_running:
            # --- Process Pygame Events (Controller Input, Window Close) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT: # User clicked window close
                    print("Window close requested - Quitting.")
                    keep_running = False
                    break
                elif event.type == pygame.KEYDOWN: # Keyboard input (less likely needed now)
                    if event.key == pygame.K_ESCAPE:
                        print("ESC key pressed - Quitting.")
                        keep_running = False
                        break
                elif event.type == pygame.JOYAXISMOTION:
                    # Check if the Right Trigger axis moved
                    if event.axis == RIGHT_TRIGGER:
                        # Axis value ranges from -1 (released) to 1 (fully pressed)
                        rt_value = joystick.get_axis(RIGHT_TRIGGER)
                        # Consider pressed if value is greater than threshold point
                        # (TRIGGER_THRESHOLD is 0-1, map it to -1 to 1 range)
                        rt_pressed = rt_value > (TRIGGER_THRESHOLD * 2 - 1)
                        # Uncomment for debugging trigger value:
                        # print(f"RT Value: {rt_value:.2f}, Pressed: {rt_pressed}")

                # Optional: Add button handling here if needed for other actions
                # elif event.type == pygame.JOYBUTTONDOWN:
                #     print(f"Button {event.button} pressed")
                # elif event.type == pygame.JOYBUTTONUP:
                #     print(f"Button {event.button} released")

            if not keep_running: break # Exit loop if flag was set by events

            # --- Get Current Controller State ---
            # Note: Pygame Y axes are often inverted (-1 is up)
            # We negate them so positive value means up/forward for Tello commands
            left_y = -scale_axis(joystick.get_axis(LEFT_STICK_Y)) # Up/Down
            left_x = scale_axis(joystick.get_axis(LEFT_STICK_X))  # Left/Right (for Yaw)
            right_y = -scale_axis(joystick.get_axis(RIGHT_STICK_Y))# Forward/Backward
            # right_x = scale_axis(joystick.get_axis(RIGHT_STICK_X)) # Strafe (if needed)

            # --- Tello Control Logic based on Right Trigger (RT) ---
            if rt_pressed:
                # --- Takeoff ---
                if not is_flying:
                    print("RT pressed - Taking off!")
                    try:
                        # Send takeoff and wait slightly for it to initiate
                        tello.takeoff()
                        is_flying = True
                        time.sleep(1.0) # Give Tello time to stabilize after takeoff
                    except Exception as e:
                        print(f"Takeoff failed: {e}")
                        # Reset flag if takeoff failed to prevent immediate retry loops
                        rt_pressed = False
                        # Optional: Maybe add a longer delay here

                # --- Flying ---
                elif is_flying:
                    # Map sticks to RC command based on user request:
                    # Left Stick Up/Down -> ud (Up/Down velocity)
                    # Left Stick Left/Right -> yaw (Yaw velocity)
                    # Right Stick Up/Down -> fb (Forward/Backward velocity)
                    # Right Stick Left/Right -> lr (Left/Right velocity - set to 0 if unused)

                    lr_speed = 0 # No strafe requested with right stick L/R
                    fb_speed = right_y
                    ud_speed = left_y
                    yaw_speed = left_x

                    # Send the command only if needed (or periodically)
                    # Sending too rapidly can overwhelm the Tello or network
                    current_time = time.time()
                    if current_time - last_rc_command_time > 0.05: # Send ~20 times/sec
                       try:
                            tello.send_rc_control(lr_speed, fb_speed, ud_speed, yaw_speed)
                            last_rc_command_time = current_time
                       except Exception as e:
                           print(f"Error sending RC command: {e}")
                           # Consider landing or hovering if commands fail repeatedly

            else: # Right Trigger is RELEASED
                # --- Landing ---
                if is_flying:
                    print("RT released - Landing!")
                    try:
                        # Stop any movement first
                        tello.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.1) # Small pause before land command
                        tello.land()
                        is_flying = False
                        # No need to constantly send land, just once is enough
                    except Exception as e:
                        print(f"Landing command failed: {e}")
                        # Drone might still be airborne, state is uncertain
                else:
                    # If on the ground and RT released, ensure it sends zero commands
                    # (Prevents leftover commands from previous run if script restarted quickly)
                    try:
                         tello.send_rc_control(0, 0, 0, 0)
                    except Exception as e:
                         print(f"Error sending zero RC command while landed: {e}")


            # --- Display Video Feed and Telemetry ---
            local_frame_copy = None
            with frame_lock:
                if video_frame is not None:
                    local_frame_copy = video_frame.copy() # Work with a copy safely

            if local_frame_copy is not None:
                # Add telemetry overlay to the frame
                display_frame = display_telemetry(local_frame_copy)
                # Show the frame in an OpenCV window
                cv2.imshow("Tello Control - Press 'q' to quit", display_frame)
            else:
                # Show a placeholder if no frame is available yet
                placeholder = np.zeros((720, 960, 3), dtype=np.uint8) # Tello standard res
                cv2.putText(placeholder, "Connecting to video stream...", (50, 360),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tello Control - Press 'q' to quit", placeholder)

            # --- Check for Keyboard Quit ('q') ---
            # Crucial: cv2.waitKey() is needed for OpenCV windows to update and process events
            key = cv2.waitKey(1) & 0xFF # Wait 1ms for key press, mask to get ASCII value
            if key == ord('q'):
                print("'q' key pressed - Quitting.")
                keep_running = False
                break # Exit the main loop immediately

            # Small delay to prevent the loop from hogging CPU excessively
            # Adjust if control feels sluggish or too responsive
            time.sleep(0.01)

    except Exception as e:
        print(f"An error occurred in the main control loop: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
    finally:
        # --- Cleanup Actions ---
        print("\nInitiating cleanup...")
        keep_running = False # Ensure flag is set for video thread

        if tello: # Check if tello object exists
            if is_flying:
                print("Ensuring Tello lands safely...")
                try:
                    # Attempt to stop movement and land multiple times if needed
                    for _ in range(3):
                        tello.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.1)
                        tello.land()
                        time.sleep(0.5)
                        # Check if still flying (might not be reliable if connection lost)
                        # if not tello.is_flying: break # Exit loop if landed
                    print("Land command sent.")
                except Exception as e:
                    print(f"Error during final landing attempt: {e}")

            print("Turning off video stream...")
            try:
                tello.streamoff()
            except Exception as e:
                print(f"Error stopping stream: {e}")

            print("Disconnecting from Tello...")
            try:
                tello.end() # Disconnects and cleans up resources
            except Exception as e:
                print(f"Error during Tello end(): {e}")

        # Wait for the video thread to finish
        print("Waiting for video thread to stop...")
        if 'video_thread' in locals() and video_thread.is_alive():
             video_thread.join(timeout=2) # Wait up to 2 seconds
             if video_thread.is_alive():
                 print("Warning: Video thread did not stop cleanly.")

        # Close OpenCV window
        print("Closing windows...")
        cv2.destroyAllWindows()

        # Quit Pygame
        print("Quitting Pygame...")
        pygame.quit()

        print("Cleanup complete. Exiting program.")

# --- Start the Application ---
if __name__ == "__main__":
    print("--- Tello Xbox Controller Script ---")
    if initialize_joystick():
        run_tello_control()
    else:
        print("Joystick initialization failed. Please ensure a controller is connected.")
        print("Exiting.")