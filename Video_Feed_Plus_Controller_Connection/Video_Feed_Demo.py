import cv2
import numpy as np
from djitellopy import Tello
import time
import threading
import traceback

# --- Global Variables ---
tello = None
keep_running = True
video_frame = None
frame_lock = threading.Lock()

# --- Helper Functions ---
def video_stream_handler():
    """Continuously fetches video frames from Tello in a separate thread."""
    global video_frame, keep_running, tello
    print("Video stream handler started.")
    frame_read = None
    if tello:
        try:
            # Get the frame reader object from Tello
            frame_read = tello.get_frame_read()
            if frame_read is None:
                print("Error: Failed to get frame reader from Tello.")
                return # Exit thread if frame reader isn't available
        except Exception as e:
            print(f"Error getting frame reader: {e}")
            return

    while keep_running:
        if not tello or not frame_read:
            # Wait if Tello is not connected or frame reader failed
            time.sleep(0.1)
            continue
        try:
            # Read the current frame
            current_frame = frame_read.frame
            if current_frame is not None:
                # Use a lock to safely update the global frame variable
                with frame_lock:
                    video_frame = current_frame.copy()
            else:
                # Small delay if no frame is available yet
                time.sleep(0.01)
        except Exception as e:
            print(f"Error in video stream handler: {e}")
            # Consider stopping if errors persist, or just log and continue
            time.sleep(0.5) # Longer delay on error

    print("Video stream handler stopping.")


# --- Main Display Function ---
def run_tello_viewer():
    """Initializes Tello, starts video stream, and displays it."""
    global keep_running, tello, video_frame

    # --- Initialize Tello ---
    tello = Tello()
    try:
        print("Connecting to Tello...")
        tello.connect()
        print("Tello connected.")
        print(f"Battery: {tello.get_battery()}%") # Still useful info

        # --- Start Video Stream ---
        print("Starting video stream...")
        tello.streamon()
        print("Video stream started.")
        # Add a small delay AFTER streamon, sometimes needed for stream init
        time.sleep(1.0)

    except Exception as e:
        print(f"Error during Tello initialization: {e}")
        traceback.print_exc()
        keep_running = False # Signal threads to stop
        return # Exit if initialization fails

    # --- Start Video Handler Thread ---
    video_thread = threading.Thread(target=video_stream_handler, daemon=True)
    video_thread.start()
    print("Waiting for first video frame...")
    time.sleep(1.5) # Give the video thread time to capture the first frame

    # --- Main Display Loop ---
    print("\n--- Tello Video Feed ---")
    print("Press 'q' in the video window to quit.")

    try:
        while keep_running:
            # --- Get the latest frame ---
            local_frame_copy = None
            with frame_lock:
                if video_frame is not None:
                    local_frame_copy = video_frame.copy()

            # --- Display the frame ---
            if local_frame_copy is not None:
                cv2.imshow("Tello Video Feed - Press 'q' to quit", local_frame_copy)
            else:
                # Show a placeholder if video stream hasn't provided a frame yet
                placeholder = np.zeros((720, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for video...", (50, 360),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tello Video Feed - Press 'q' to quit", placeholder)

            # --- Check for Keyboard Quit ('q') ---
            # waitKey waits for specified ms for a key event.
            # Essential for cv2.imshow() to work correctly.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("'q' key pressed - Quitting.")
                keep_running = False # Signal threads and loop to stop
                break

            # Small delay to prevent high CPU usage
            time.sleep(0.01)

    except Exception as e:
        print(f"\n--- UNHANDLED ERROR IN MAIN LOOP ---")
        print(f"Error: {e}")
        traceback.print_exc()
        keep_running = False # Ensure cleanup happens on error

    finally:
        # --- Cleanup Actions ---
        print("\nInitiating cleanup...")
        keep_running = False # Explicitly ensure flag is set

        if tello:
            print("Turning off video stream...")
            try:
                tello.streamoff()
            except Exception as e:
                print(f"Error stopping stream: {e}")

            print("Disconnecting from Tello...")
            try:
                tello.end() # Closes the connection
            except Exception as e:
                print(f"Error during Tello end(): {e}")

        print("Waiting for video thread to stop...")
        if 'video_thread' in locals() and video_thread.is_alive():
             video_thread.join(timeout=2) # Wait max 2 seconds for thread
             if video_thread.is_alive():
                 print("Warning: Video thread did not stop cleanly.")

        print("Closing OpenCV windows...")
        cv2.destroyAllWindows()
        print("Cleanup complete. Exiting program.")


# --- Start the Application ---
if __name__ == "__main__":
    print("--- Tello Video Viewer ---")
    run_tello_viewer()
    print("Application finished.")