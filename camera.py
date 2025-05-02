import cv2
from pyzbar.pyzbar import decode
import time

_last_qr = None
_last_emotion = None
qr_detected_callback = None
emotion_callback = None

def classify_emotion(frame):
    return "neutral" # Replace with actual model

def run_camera(qr_callback, emotion_callback_fn, cam_index=0):
    global _last_qr, _last_emotion, qr_detected_callback, emotion_callback
    qr_detected_callback = qr_callback
    emotion_callback = emotion_callback_fn

    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW) # Try CAP_MSMF if DSHOW fails

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        # QR detection
        codes = decode(frame)
        if codes:
            qr = codes[0]
            text = qr.data.decode('utf-8')
            center = tuple(map(int, [sum(p.x for p in qr.polygon)/len(qr.polygon),
                                     sum(p.y for p in qr.polygon)/len(qr.polygon)]))
            if text != _last_qr:
                _last_qr = text
                qr_detected_callback(text, center)

        # Emotion (limited to 1 Hz)
        if int(time.time()) % 1 == 0:
            emotion = classify_emotion(frame)
            if emotion != _last_emotion:
                _last_emotion = emotion
                emotion_callback(emotion)

        # Show camera window
        cv2.imshow("View", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
