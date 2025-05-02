# camera_win.py
import cv2
from pyzbar.pyzbar import decode
import time

from deepface import DeepFace # New import

_last_qr = None
_last_emotion = None
qr_detected_callback = None
emotion_callback = None

def classify_emotion(frame):
    try:
        # DeepFace expects RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = DeepFace.analyze(
            rgb,
            actions=['emotion'],
            enforce_detection=False, # Won't crash if no face
            detector_backend='opencv'
        )
        return result[0]['dominant_emotion']
    except Exception as e:
        print("Emotion analysis failed:", e)
        return "neutral"

def run_camera(qr_callback, emotion_callback_fn, cam_index=0):
    """
    Captures frames, publishes unique QR scans and emotion changes.
    qr_callback(text, center): called on new QR string
    emotion_callback_fn(emotion_str): called on new emotion
    """
    global _last_qr, _last_emotion, qr_detected_callback, emotion_callback
    qr_detected_callback = qr_callback
    emotion_callback = emotion_callback_fn

    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)

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
            center = (
                int(sum(p.x for p in qr.polygon) / len(qr.polygon)),
                int(sum(p.y for p in qr.polygon) / len(qr.polygon))
            )
            if text != _last_qr:
                _last_qr = text
                qr_detected_callback(text, center)

        # Emotion detection (limit update to once per second)
        now = time.time()
        if int(now) != int(now - 0.2):  # simple 1 Hz gate
            emotion = classify_emotion(frame)
            if emotion != _last_emotion:
                _last_emotion = emotion
                emotion_callback(emotion)

        # Debug display
        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
