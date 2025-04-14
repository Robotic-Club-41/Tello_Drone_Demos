import pygame
import cv2
import numpy as np
from djitellopy import Tello
import time
import threading
import math
import traceback

# --- Configuration ---
CONTROL_SENSITIVITY = 75
AXIS_DEADZONE = 0.15
TRIGGER_THRESHOLD = 0.1 # Range -1 (released) to 1 (pressed)
LANDED_TOF_THRESHOLD = 20 # cm - Altitude below which we consider the drone landed

# --- Controller Mapping ---
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
RIGHT_TRIGGER = 5

# --- Global Variables ---
tello = None
keep_running = True
is_flying = False
video_frame = None
frame_lock = threading.Lock()
joystick = None
rt_pressed = False
was_rt_pressed = False
needs_to_land = False

# --- Helper Functions ---
# initialize_joystick, scale_axis, video_stream_handler remain the same as previous version
# ... (paste the initialize_joystick, scale_axis, video_stream_handler functions here) ...
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
            return

    while keep_running:
        if not tello or not frame_read:
            time.sleep(0.1)
            continue
        try:
            current_frame = frame_read.frame
            if current_frame is not None:
                with frame_lock:
                    video_frame = current_frame.copy()
            else:
                time.sleep(0.01)
        except Exception as e:
            # print(f"Error in video stream handler: {e}") # Can be noisy
            time.sleep(0.1)

    print("Video stream handler stopping.")


def display_telemetry(frame):
    """Gets telemetry data and draws it onto the provided frame."""
    global tello, is_flying, rt_pressed, needs_to_land

    if frame is None or tello is None:
        return frame

    display_frame = frame.copy()

    try:
        tof_distance = tello.get_distance_tof()
        speed_x = tello.get_speed_x()
        speed_y = tello.get_speed_y()
        speed_z = tello.get_speed_z()
        yaw = tello.get_yaw()
        battery = tello.get_battery()
        flight_time = tello.get_flight_time()
        temp = tello.get_temperature() # Tello temperature

        horizontal_speed = math.sqrt(speed_x**2 + speed_y**2)

        status = "Landed"
        if is_flying:
            if rt_pressed: status = "Flying (Armed)"
            elif needs_to_land: status = "Landing..."
            else: status = "Flying (Disarmed?)" # Should be brief state
        elif rt_pressed and not is_flying: status = "Takeoff Pending..."
        elif needs_to_land: status = "Landing..." # If needs_to_land is still true after RT release

        # Temperature Warning - Tello drones (esp. original) get hot! This is often normal.
        if temp > 85: # Slightly increased threshold, 80 is quite common
            status += " HIGH TEMP!"
        elif temp > 80:
             status += " Temp Warm"


        if battery < 15: status += " LOW BATT!"

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
            # Draw shadow/outline first for better readability
            cv2.putText(display_frame, line, (11, y_pos + 1), font, font_scale, shadow_color, shadow_thickness, line_type)
            # Draw text
            cv2.putText(display_frame, line, (10, y_pos), font, font_scale, color, thickness, line_type)
            y_pos += 25

        return display_frame

    except Exception as e:
        # print(f"Telemetry Error: {e}") # Can be noisy if happens often
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
        tello.send_rc_control(0, 0, 0, 0)
        print("Starting video stream...")
        tello.streamon()
        print("Video stream started.")
        tello.set_speed(60)
        print("Speed set to 60.")
        # Add a small delay AFTER streamon, sometimes needed
        time.sleep(0.5)
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
    time.sleep(1.5) # Reduced wait time

    # --- Main Loop ---
    print("\n--- Ready to Fly ---")
    print("Hold Right Trigger (RT) to take off and fly.")
    print("Release Right Trigger (RT) to land.")
    print("Press 'q' in the video window to quit.")
    print(f"(Note: High temp warnings are common for Tello, allow cooldown.)") # Added note

    last_rc_command_time = time.time()
    rc_command = [0, 0, 0, 0]

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
            if not keep_running: break

            # --- Get Controller State ---
            try:
                rt_value = joystick.get_axis(RIGHT_TRIGGER)
                rt_pressed = rt_value > (TRIGGER_THRESHOLD * 2 - 1)
            except Exception as e:
                print(f"Error reading trigger axis: {e}. Assuming released for safety.")
                rt_pressed = False
                if is_flying: needs_to_land = True # Land if trigger read fails while flying

            # Get stick values only if needed
            if rt_pressed and is_flying:
                try:
                    left_y = -scale_axis(joystick.get_axis(LEFT_STICK_Y))
                    left_x = scale_axis(joystick.get_axis(LEFT_STICK_X))
                    right_y = -scale_axis(joystick.get_axis(RIGHT_STICK_Y))
                    rc_command = [0, right_y, left_y, left_x] # lr, fb, ud, yaw
                except Exception as e:
                     print(f"Error reading stick axes: {e}. Sending zero command.")
                     rc_command = [0, 0, 0, 0]
                     if is_flying: needs_to_land = True # Land if stick read fails
            else:
                 rc_command = [0, 0, 0, 0]


            # --- State Machine Logic ---

            # 1. Takeoff: Trigger pressed now, wasn't pressed before, and drone is not flying
            if rt_pressed and not was_rt_pressed and not is_flying:
                print("Takeoff initiated...")
                try:
                    # Optional: Check battery before takeoff
                    # if tello.get_battery() < 15:
                    #    print("Takeoff aborted: Battery low!")
                    # else:
                    tello.takeoff()
                    is_flying = True
                    needs_to_land = False # Explicitly false on takeoff
                    print("Takeoff successful.")
                except Exception as e:
                    print(f"Takeoff failed: {e}")
                    is_flying = False # Ensure state reflects failure

            # 2. Initiate Landing: Trigger released now, was pressed before, and drone is flying
            elif not rt_pressed and was_rt_pressed and is_flying:
                print("Landing initiated (Trigger Released)...")
                needs_to_land = True
                rc_command = [0, 0, 0, 0] # Ensure zero command is set
                try:
                    tello.send_rc_control(0, 0, 0, 0) # Send zero command immediately
                    time.sleep(0.05) # Short delay before land command
                    tello.land()
                except Exception as e:
                    print(f"Land command failed: {e}")
                    # Keep needs_to_land=True, hopefully zero commands still work

            # 3. Flying: Trigger pressed AND drone is flying
            if rt_pressed and is_flying:
                # Send RC commands periodically
                if current_time - last_rc_command_time > 0.05: # ~20Hz
                   try:
                        tello.send_rc_control(rc_command[0], rc_command[1], rc_command[2], rc_command[3])
                        last_rc_command_time = current_time
                   except Exception as e:
                       print(f"Error sending RC command while flying: {e}")
                       needs_to_land = True # Initiate landing on command error

            # 4. Landing / Grounded and Trigger Released:
            elif not rt_pressed and (is_flying or needs_to_land):
                 # Send zero commands continuously while landing or grounded with RT released
                 if current_time - last_rc_command_time > 0.05:
                    try:
                        tello.send_rc_control(0, 0, 0, 0)
                        last_rc_command_time = current_time

                        # *** LANDING DETECTION LOGIC ***
                        if needs_to_land: # Only check if landing was initiated
                             try:
                                 tof = tello.get_distance_tof()
                                 # print(f"Debug ToF: {tof}") # Uncomment for debugging landing detection
                                 if tof <= LANDED_TOF_THRESHOLD:
                                     print(f"Detected landed (ToF: {tof} cm). Ready for next takeoff.")
                                     is_flying = False
                                     needs_to_land = False
                                     # No need to send more land commands, just keep sending 0 RC
                             except Exception as e:
                                 # Error reading ToF during landing check isn't critical,
                                 # just means we might not reset is_flying automatically.
                                 # print(f"Warning: Could not read ToF during landing check: {e}")
                                 pass # Continue sending zero commands

                    except Exception as e:
                        print(f"Error sending Zero RC command while landing/grounded: {e}")


            # Update previous trigger state for the next loop iteration
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
                # Keep showing placeholder if video not ready
                placeholder = np.zeros((720, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for video...", (50, 360), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tello Control - Press 'q' to quit", placeholder)


            # --- Check for Keyboard Quit ('q') ---
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("'q' key pressed - Quitting.")
                keep_running = False
                if is_flying and not needs_to_land: # Initiate landing if flying
                    needs_to_land = True
                    try:
                        print("Landing due to quit command...")
                        tello.send_rc_control(0,0,0,0)
                        time.sleep(0.1)
                        tello.land()
                    except Exception as e: print(f"Error sending land on quit: {e}")
                break

            # Loop timing control
            time.sleep(0.01)


    except Exception as e:
        print(f"\n--- UNHANDLED ERROR IN MAIN LOOP ---")
        print(f"Error: {e}")
        traceback.print_exc()
        keep_running = False
        if tello and is_flying: needs_to_land = True # Trigger landing in finally block

    finally:
        # --- Cleanup Actions --- (same as before)
        # ... (paste the finally block here) ...
        print("\nInitiating cleanup...")
        keep_running = False

        if tello:
            if is_flying or needs_to_land: # Check if it was airborne or trying to land
                print("Ensuring Tello lands...")
                try:
                    tello.send_rc_control(0, 0, 0, 0) # Stop movement
                    time.sleep(0.1)
                    # Send land command multiple times for robustness
                    for _ in range(3):
                        try: tello.land()
                        except Exception: pass # Ignore errors here, just try again
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
        print("Joystick initialization failed. Please ensure controller is connected.")
        print("Exiting.")