from naoqi import ALProxy
import tf2_ros  # optional if you compute 3D from pose

class GestureBehavior(object):
    def __init__(self, session):
        self.mem    = session.service("ALMemory")
        self.motion = session.service("ALMotion")
        self.mem.subscribeToEvent("Game/LastQR", "GestureBehavior", "onCard")

    def onCard(self, key, value, msg):
        # simple arm point: open hand and extend
        effector = "RArm"
        self.motion.openHand(effector)
        # point by setting shoulder pitch/roll
        self.motion.setAngles("RShoulderPitch", 0.5, 0.1)
        self.motion.setAngles("RShoulderRoll", -0.2, 0.1)
        # after a pause, return to rest
        time.sleep(1.0)
        self.motion.rest()  
