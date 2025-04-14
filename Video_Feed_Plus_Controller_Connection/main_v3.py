import pygame
import cv2
import numpy as np
from djitellopy import Tello
import time
import threading
import math
import traceback # Added for better error reporting

# --- Configuration ---
CONTROL_SENSITIVITY = 75
AXIS_DEADZONE = 0.15
TRIGGER_THRESHOLD = 0.1 # Range -1 (released) to 1 (pressed)

# --- Controller Mapping ---
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5

# --- Global Variables ---
tello = None
keep_running = True
is_flying = False # Represents if takeoff command has succeeded
video_frame = None
frame_lock = threading.Lock()
joystick = None
rt_pressed = False          # Current state of the right trigger
was_rt_pressed = False      # State of RT in the previous loop iteration
needs_to_land = False       # Flag to signal landing sequence initiation

# --- Helper Functions --- (initialize_joystick, scale_axis, video_stream_handler, display_telemetry remain largely the same)
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
        print(f"  Axes: {joystick.get_numaxes()}")
        print(f"  Buttons: {joystick.get_numbuttons()}")
        # Verify required axes exist
        required_axes = [LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y, RIGHT_TRIGGER]
        if joystick.get_numaxes() < max(required_axes) + 1:
            print(f"Error: Joystick has only {joystick.get_numaxes()} axes. Need at least {max(required_axes) + 1}.")
            pygame.quit()
            return False
        return True

def scale_axis(value):
    """Scales joystick axis value (-1 to 1) to Tello command value (-100 to 100)
       applying deadzone and sensitivity."""
    if abs(value) < AXIS_DEADZONE:
        return 0
    if value > 0:
        scaled_value = (value - AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    else:
        scaled_value = (value + AXIS_DEADZONE) / (1 - AXIS_DEADZONE)
    return int(scaled_value * CONTROL_SENSITIVITY)

def video_stream_handler():
    """Continuously fetches video frames from Tello in a separate thread."""
    global video_frame, keep_running, tello
    print("Video stream handler started.")
    frame_read = None
    if tello:
        try:
            frame_read = tello.get_frame_read()
        except Exception as e:
            print(f"Error getting frame reader: {e}")
            # Consider setting keep_running = False if video is critical
            return

    while keep_running:
        if not tello or not frame_read:
            time.sleep(0.1) # Wait if tello object is not ready
            continue
        try:
            current_frame = frame_read.frame
            if current_frame is not None:
                with frame_lock:
                    video_frame = current_frame.copy()
            else:
                time.sleep(0.01) # Avoid busy-waiting if frame is temporarily None
        except Exception as e:
            print(f"Error in video stream handler: {e}")
            # traceback.print_exc() # Uncomment for detailed error
            time.sleep(0.1)

    print("Video stream handler stopping.")


def display_telemetry(frame):
    """Gets telemetry data and draws it onto the provided frame."""
    global tello, is_flying, rt_pressed, needs_to_land # Make sure globals are accessible

    if frame is None or tello is None:
        return frame # Nothing to draw on or no tello object

    display_frame = frame.copy()

    try:
        tof_distance = tello.get_distance_tof()
        speed_x = tello.get_speed_x()
        speed_y = tello.get_speed_y()
        speed_z = tello.get_speed_z()
        yaw = tello.get_yaw()
        battery = tello.get_battery()
        flight_time = tello.get_flight_time()
        temp = tello.get_temperature()

        horizontal_speed = math.sqrt(speed_x**2 + speed_y**2)

        # Determine flight status string more accurately
        status = "Landed"
        if is_flying:
            if rt_pressed:
                status = "Flying (Armed)"
            elif needs_to_land:
                status = "Landing..."
            else:
                 status = "Flying (Disarmed - Glide?)" # Should ideally not happen long
        elif rt_pressed and not is_flying:
             status = "Takeoff Pending..."
        elif not rt_pressed and needs_to_land: # Still landing after release
             status = "Landing..."

        if battery < 15: status += " LOW BATT!"
        if temp > 80: status += " HIGH TEMP!"

        text_lines = [
            f"Status: {status}",
            f"Altitude (ToF): {tof_distance} cm",
            f"Speed H: {horizontal_speed:.1f} V: {speed_z:.1f} cm/s",
            f"Yaw (Rel): {yaw} deg",
            f"Battery: {battery}% Temp: {temp}C",
            f"Flight Time: {flight_time}s",
        ]

        y_pos = 30
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)
        shadow_color = (0, 0, 0)
        thickness = 1
        shadow_thickness = 2
        line_type = cv2.LINE_AA

        for line in text_lines:
            cv2.putText(display_frame, line, (11, y_pos + 1), font, font_scale, shadow_color, shadow_thickness, line_type)
            cv2.putText(display_frame, line, (10, y_pos), font, font_scale, color, thickness, line_type)
            y_pos += 25

        return display_frame

    except Exception as e:
        print(f"Telemetry Error: {e}")
        cv2.putText(display_frame, "Telemetry Error", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        return display_frame


# --- Main Control Function ---
def run_tello_control():
    global keep_running, is_flying, tello, video_frame, rt_pressed, was_rt_pressed, needs_to_land

    # --- Initialize Tello ---
    tello = Tello()
    try:
        print("Connecting to Tello...")
        tello.connect()
        print("Tello connected.")
        print(f"Battery: {tello.get_battery()}%")
        tello.send_rc_control(0, 0, 0, 0) # Reset commands
        print("Starting video stream...")
        tello.streamon()
        print("Video stream started.")
        tello.set_speed(60)
        print("Speed set to 60.")
    except Exception as e:
        print(f"Error during Tello initialization: {e}")
        traceback.print_exc()
        keep_running = False
        if joystick: pygame.quit()
        return

    # --- Start Video Handler Thread ---
    video_thread = threading.Thread(target=video_stream_handler, daemon=True)
    video_thread.start()
    print("Waiting for video stream...")
    time.sleep(2.0) # Allow time for connection and first frame

    # --- Main Loop ---
    print("\n--- Ready to Fly ---")
    print("Hold Right Trigger (RT) to take off and fly.")
    print("Release Right Trigger (RT) to land.")
    print("Press 'q' in the video window to quit.")

    last_rc_command_time = time.time()
    rc_command = [0, 0, 0, 0] # Initialize rc command list

    try:
        while keep_running:
            current_time = time.time()

            # --- Process Pygame Events ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("Window close requested - Quitting.")
                    keep_running = False
                    break
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("ESC key pressed - Quitting.")
                        keep_running = False
                        break
                # Check Axis Motion *reliably* for the trigger
                # It's better to check axis value directly in loop than rely solely on event
                # But event processing is still needed for window close, keyboard etc.

            if not keep_running: break

            # --- Get Controller State ---
            # Check trigger state EVERY loop iteration
            try:
                rt_value = joystick.get_axis(RIGHT_TRIGGER)
                rt_pressed = rt_value > (TRIGGER_THRESHOLD * 2 - 1) # Map 0..1 threshold to -1..1 axis
            except Exception as e:
                print(f"Error reading trigger axis: {e}")
                # Safety: Assume trigger released if reading fails
                rt_pressed = False
                # Consider landing if flying? Or just stop commands?
                if is_flying: needs_to_land = True

            # Get stick values IF trigger is pressed and flying
            if rt_pressed and is_flying:
                try:
                    left_y = -scale_axis(joystick.get_axis(LEFT_STICK_Y)) # Up/Down
                    left_x = scale_axis(joystick.get_axis(LEFT_STICK_X))  # Yaw
                    right_y = -scale_axis(joystick.get_axis(RIGHT_STICK_Y))# Forward/Backward
                    # right_x = scale_axis(joystick.get_axis(RIGHT_STICK_X)) # Strafe (unused)

                    rc_command = [0, right_y, left_y, left_x] # lr, fb, ud, yaw
                except Exception as e:
                     print(f"Error reading stick axes: {e}")
                     # Safety: Send zero commands if sticks fail
                     rc_command = [0, 0, 0, 0]
                     # Consider landing
                     needs_to_land = True
            else:
                 # Ensure commands are zeroed otherwise
                 rc_command = [0, 0, 0, 0]


            # --- State Machine Logic ---

            # 1. Takeoff Condition: Trigger pressed, was not pressed before, not currently flying
            if rt_pressed and not was_rt_pressed and not is_flying:
                print("Takeoff initiated...")
                try:
                    tello.takeoff()
                    is_flying = True
                    needs_to_land = False # Reset landing flag on successful takeoff
                    print("Takeoff successful.")
                    # NO time.sleep() here!
                except Exception as e:
                    print(f"Takeoff failed: {e}")
                    is_flying = False # Ensure state reflects failure

            # 2. Landing Condition: Trigger released, was pressed before, currently flying
            elif not rt_pressed and was_rt_pressed and is_flying:
                print("Landing initiated (Trigger Released)...")
                needs_to_land = True
                # Send the land command *once*
                try:
                    tello.land()
                    # Note: Don't set is_flying = False here yet. Drone is still airborne.
                    # Keep sending zero commands while landing.
                except Exception as e:
                    print(f"Land command failed: {e}")
                    # State might be uncertain, but keep trying to send zero commands

            # 3. Flying: Trigger pressed AND drone is flying
            if rt_pressed and is_flying:
                # Send RC control commands derived from sticks
                # Send periodically, not necessarily every single loop iteration if loop is very fast
                if current_time - last_rc_command_time > 0.05: # ~20Hz
                   try:
                        tello.send_rc_control(rc_command[0], rc_command[1], rc_command[2], rc_command[3])
                        last_rc_command_time = current_time
                   except Exception as e:
                       print(f"Error sending RC command while flying: {e}")
                       # Consider emergency landing or hover on repeated errors
                       needs_to_land = True # Initiate landing on command error

            # 4. Landing / Grounded and Trigger Released:
            elif not rt_pressed and (is_flying or needs_to_land):
                 # If we are in the landing phase OR just on ground with trigger released,
                 # keep sending zero commands to ensure stability / prevent drift.
                 # Send more frequently than flying commands perhaps? Or same rate.
                 if current_time - last_rc_command_time > 0.05:
                    try:
                        tello.send_rc_control(0, 0, 0, 0)
                        last_rc_command_time = current_time
                        # We don't really know when landing *finishes* easily from SDK.
                        # We could try checking altitude, but ToF is noisy near ground.
                        # For simplicity, rely on user to press trigger again for next takeoff.
                        # If drone seems landed (low ToF?), maybe set is_flying=False?
                        # Example (optional):
                        # tof = tello.get_distance_tof()
                        # if is_flying and tof < 20: # Arbitrary threshold for landed
                        #      print("Looks landed based on ToF.")
                        #      is_flying = False # Reset state
                        #      needs_to_land = False
                    except Exception as e:
                        print(f"Error sending Zero RC command while landing/grounded: {e}")
                        # If this fails, drone might drift

            # Update previous trigger state for next iteration's comparison
            was_rt_pressed = rt_pressed

            # --- Display Video Feed and Telemetry ---
            local_frame_copy = None
            with frame_lock:
                if video_frame is not None:
                    local_frame_copy = video_frame.copy()

            if local_frame_copy is not None:
                display_frame = display_telemetry(local_frame_copy)
                cv2.imshow("Tello Control - Press 'q' to quit", display_frame)
            else:
                placeholder = np.zeros((720, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for video...", (50, 360),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tello Control - Press 'q' to quit", placeholder)

            # --- Check for Keyboard Quit ('q') ---
            key = cv2.waitKey(1) & 0xFF # Essential for OpenCV window updates
            if key == ord('q'):
                print("'q' key pressed - Quitting.")
                keep_running = False
                # Initiate landing if flying when 'q' is pressed
                if is_flying and not needs_to_land:
                    needs_to_land = True
                    try:
                        print("Landing due to quit command...")
                        tello.send_rc_control(0,0,0,0) # Stop first
                        time.sleep(0.1)
                        tello.land()
                    except Exception as e:
                        print(f"Error sending land on quit: {e}")
                break

            # Loop timing control (adjust if needed)
            time.sleep(0.01) # Small sleep to prevent 100% CPU usage


    except Exception as e:
        print(f"\n--- UNHANDLED ERROR IN MAIN LOOP ---")
        print(f"Error: {e}")
        traceback.print_exc()
        print("------------------------------------")
        keep_running = False
        # Try emergency landing
        if tello and is_flying:
             print("Attempting emergency land due to error...")
             needs_to_land = True # Signal cleanup to land

    finally:
        # --- Cleanup Actions ---
        print("\nInitiating cleanup...")
        keep_running = False # Ensure flag is set for video thread

        if tello:
            # Ensure landing if it was flying or intended to land
            # Check tello.is_flying maybe? Be careful, it might be unreliable if disconnected
            if is_flying or needs_to_land:
                print("Ensuring Tello lands...")
                try:
                    # Send land multiple times just in case
                    for _ in range(3):
                        tello.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.1)
                        tello.land()
                        time.sleep(0.5)
                    print("Final land command sent.")
                except Exception as e:
                    print(f"Error during final landing attempt: {e}")

            print("Turning off video stream...")
            try: tello.streamoff()
            except Exception as e: print(f"Error stopping stream: {e}")

            print("Disconnecting from Tello...")
            try: tello.end()
            except Exception as e: print(f"Error during Tello end(): {e}")

        print("Waiting for video thread to stop...")
        if 'video_thread' in locals() and video_thread.is_alive():
             video_thread.join(timeout=2)
             if video_thread.is_alive(): print("Warning: Video thread did not stop cleanly.")

        print("Closing windows...")
        cv2.destroyAllWindows()
        print("Quitting Pygame...")
        pygame.quit()
        print("Cleanup complete. Exiting program.")


# --- Start the Application ---
if __name__ == "__main__":
    print("--- Tello Xbox Controller Script ---")
    if initialize_joystick():
        run_tello_control()
    else:
        print("Joystick initialization failed. Please ensure controller is connected and drivers are working.")
        print("Exiting.")