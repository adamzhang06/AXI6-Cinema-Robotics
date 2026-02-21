import cv2

def main():
    print("Initializing Logitech Brio 101...")
    
    # 0 is usually the default index for the first USB camera on a Pi.
    # If it fails, sometimes the Pi assigns it to 1 or 2 depending on the USB bus.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("ERROR: Cannot open camera. Check the USB connection.")
        return

    # Keep the resolution low (640x480) for the initial test. 
    # The Pi 3 can struggle with 1080p live previews.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Camera active! Press 'q' to quit.")

    while True:
        success, frame = cap.read()
        
        if not success:
            print("ERROR: Failed to pull a frame from the camera.")
            break

        # Show the live feed on your monitor
        cv2.imshow("Pi 3 Camera Test", frame)

        # Listen for the 'q' key to cleanly exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Safely release the hardware
    cap.release()
    cv2.destroyAllWindows()
    print("Test concluded. Hardware released.")

if __name__ == "__main__":
    main()