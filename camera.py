#!/usr/bin/env python
import rospy
import cv2
import yaml
import numpy as np
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import tf2_ros
import tf.transformations as tf_trans

def load_camera_calib():
    try:
        data = rospy.get_param('/camera_calibration')
        K = np.array(data['camera_matrix']['data']).reshape(3, 3)
        D = np.array(data['dist_coeffs']['data'])
        return K, D
    except KeyError:
        rospy.logerr("Camera calibration not found in parameter server.")
        rospy.signal_shutdown("Missing camera calibration.")
        return None, None

def classify_emotion(frame):
    """Stub for emotion detection"""
    # You can replace this with a call to a real model
    return 'neutral'  # or 'sad', 'happy' for simulation

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        self.bridge = CvBridge()
        self.K, self.D = load_camera_calib()

        self.object_pts = self._generate_card_corners()

        self.pose_pub = rospy.Publisher('/detected_card_pose', PoseStamped, queue_size=1)
        self.emotion_pub = rospy.Publisher('/player_emotion', String, queue_size=1)

        self.setup_tf()
        self.setup_subscriber()

        rospy.loginfo("Vision node initialized.")
        rospy.spin()

    def _generate_card_corners(self):
        # A5 card dimensions (meters)
        w, h = 0.148, 0.210
        return np.array([
            [-w/2, -h/2, 0],
            [ w/2, -h/2, 0],
            [ w/2,  h/2, 0],
            [-w/2,  h/2, 0]
        ], dtype=np.float64)

    def setup_tf(self):
        tf_broad = tf2_ros.StaticTransformBroadcaster()
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'camera_link'
        static_tf.transform.translation.x = 0.5
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 1.2
        static_tf.transform.rotation.x = 0
        static_tf.transform.rotation.y = 0
        static_tf.transform.rotation.z = 0
        static_tf.transform.rotation.w = 1
        tf_broad.sendTransform(static_tf)
        rospy.loginfo("Static TF from base_link to camera_link published.")

    def setup_subscriber(self):
        topic = rospy.get_param('~image_topic', '/camera/image_raw')
        rospy.Subscriber(topic, Image, self.callback, queue_size=1)
        rospy.loginfo(f"Subscribed to image topic: {topic}")

    def callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # Emotion detection
        emotion = classify_emotion(frame)
        self.emotion_pub.publish(emotion)
        rospy.loginfo(f"Detected emotion: {emotion}")

        # QR code detection
        codes = decode(frame)
        if not codes:
            return

        for qr in codes:
            if not qr.polygon or len(qr.polygon) != 4:
                rospy.logwarn("QR code with invalid polygon skipped.")
                continue

            pts = np.array([(p.x, p.y) for p in qr.polygon], dtype=np.float64)

            ok, rvec, tvec = cv2.solvePnP(self.object_pts, pts, self.K, self.D)
            if not ok:
                rospy.logwarn("solvePnP failed.")
                continue

            # Pose message
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = 'camera_link'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = tvec.flatten().tolist()

            R, _ = cv2.Rodrigues(rvec)
            transform = np.eye(4)
            transform[:3, :3] = R
            q = tf_trans.quaternion_from_matrix(transform)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q

            self.pose_pub.publish(ps)

            data = qr.data.decode('utf-8')
            rospy.loginfo(f"Detected card: {data}")
            break  # process only one for now; remove this line for multi-card support

        # Debug (optional)
        # cv2.imshow("Camera", frame)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        VisionNode()
    except rospy.ROSInterruptException:
        pass
