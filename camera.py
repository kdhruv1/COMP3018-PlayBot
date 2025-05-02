from naoqi import ALProxy
import vision_definitions
import numpy as np
from pyzbar.pyzbar import decode
import cv2
from deepFace import classify_emotion

class CameraBehavior(object):
    def __init__(self, session):
        self.cam = session.service("ALVideoDevice")
        self.mem = session.service("ALMemory")

        # subscribe to top camera at 320x240 @ 10fps, BGR
        self.sub = self.cam.subscribe("GameCam",
                                      vision_definitions.kVGA,
                                      vision_definitions.kBGRColorSpace,
                                      10)

    def processFrame(self):
        # get image
        nao_img = self.cam.getImageRemote(self.sub)
        w, h = nao_img[0], nao_img[1]
        arr = np.frombuffer(nao_img[6], dtype=np.uint8)
        frame = arr.reshape((h, w, 3))

        # QR decoding
        codes = decode(frame)
        if codes:
            text = codes[0].data.decode('utf-8')
            self.mem.insertData("Game/LastQR", text)

        # Emotion
        emo = classify_emotion(frame)  # from deepFace.py
        self.mem.insertData("Game/LastEmotion", emo)

    def cleanup(self):
        self.cam.unsubscribe(self.sub)
