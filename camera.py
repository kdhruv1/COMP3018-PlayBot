#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import tf2_ros
import tf.transformations as tf_trans

def load_camera_calib():
    data = rospy.get_param('/camera_calibration')
    K = np.array(data['camera_matrix']['data']).reshape(3,3)
    D = np.array(data['dist_coeffs']['data'])
    return K, D

def classify_emotion(frame):
    # Dummy placeholder – replace with model inference
    return 'neutral'

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        self.tf_broad = tf2_ros.StaticTransformBroadcaster()
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
        self.tf_broad.sendTransform(static_tf)

        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('/detected_card_pose', PoseStamped, queue_size=1)
        self.text_pub = rospy.Publisher('/detected_card_text', String, queue_size=1)
        self.emotion_pub = rospy.Publisher('/player_emotion', String, queue_size=1)
        self.K, self.D = load_camera_calib()

        self.object_pts = np.array([[-0.074, -0.105, 0], [0.074, -0.105, 0],
                                    [0.074, 0.105, 0], [-0.074, 0.105, 0]], dtype=np.float64)

        topic = rospy.get_param('~image_topic', '/camera/image_raw')
        rospy.Subscriber(topic, Image, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        codes = decode(frame)
        if not codes:
            return
        qr = codes[0]
        pts = np.array([(p.x, p.y) for p in qr.polygon], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(self.object_pts, pts, self.K, self.D)
        if not ok:
            return

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'camera_link'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = tvec.flatten().tolist()
        R, _ = cv2.Rodrigues(rvec)
        q = tf_trans.quaternion_from_matrix(np.vstack((np.hstack((R, [[0],[0],[0]])), [0, 0, 0, 1])))
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q.tolist()
        self.pose_pub.publish(ps)

        # Publish readable card content
        self.text_pub.publish(String(data=qr.data.decode('utf-8')))

        # Emotion detection (stub)
        emotion = classify_emotion(frame)
        self.emotion_pub.publish(String(data=emotion))
        rospy.loginfo(f"Detected emotion: {emotion}")

if __name__ == '__main__':
    try:
        VisionNode()
    except rospy.ROSInterruptException:
        pass
