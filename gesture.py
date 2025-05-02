# pip install opencv-python pyzbar pillow
# python gesture.py
from camera_win import subscribe

def gesture_to_card(data):
    text = data['text']
    pose = data['pose']  # (x_pix, y_pix)
    print(f">>> [Gesture] Pointing at '{text}' located at pixel {pose}")

def main():
    subscribe('card', gesture_to_card)
    print("Waiting for QR scans to gesture at...")
    try:
        # Keep alive
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nGesture node stopped.")

if __name__ == '__main__':
    main()
