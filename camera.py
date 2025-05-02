import cv2
from pyzbar.pyzbar import decode
import time
from deepFace import classify_emotion # deepFace.py

_last_qr = None
_last_emotion = None
_last_em_time = 0.0
qr_detected_callback = None
emotion_callback = None

def run_camera(qr_callback, emotion_callback_fn, cam_index=0):
    global _last_qr, _last_emotion, _last_em_time
    global qr_detected_callback, emotion_callback

    qr_detected_callback = qr_callback
    emotion_callback  = emotion_callback_fn

    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW) # Or CAP_MSMF

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

        # Emotion detection at ~1 Hz
        now = time.time()
        if now - _last_em_time >= 1.0:
            _last_em_time = now
            emotion = classify_emotion(frame)
            if emotion != _last_emotion:
                _last_emotion = emotion
                emotion_callback(emotion)

        # Debug display
        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == 27: # ESC to exit
            break

    cap.release()
    cv2.destroyAllWindows()
