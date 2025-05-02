# Python camera.py
import cv2
from pyzbar.pyzbar import decode
import threading
import time

# Simple registry
_subs = {'card': [], 'emotion': []}

def subscribe(event, callback):
    """Subscribe a callback to 'card' or 'emotion' events."""
    _subs[event].append(callback)

def _publish(event, data):
    for cb in _subs.get(event, []):
        cb(data)

def classify_emotion(frame):
    """
    Emotion stub: replace with real model if you like.
    For now, toggles every 5 seconds between 'neutral' and 'sad'.
    """
    t = int(time.time())//5
    return ['neutral','sad','happy'][t % 3]

def camera_loop(cam_index=0):
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # QR code detection
        codes = decode(frame)
        if codes:
            qr = codes[0]
            text = qr.data.decode('utf-8') # Example "10 of Clubs"
            # Approximate pose = center of polygon in pixel coords
            pts = [(p.x, p.y) for p in qr.polygon]
            cx = sum(x for x,y in pts)/len(pts)
            cy = sum(y for x,y in pts)/len(pts)
            _publish('card', {'text': text, 'pose': (cx, cy)})

        # 2) Emotion classification
        emotion = classify_emotion(frame)
        _publish('emotion', emotion)

        # Show for debug
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == 27: # ESC to quit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_loop()
