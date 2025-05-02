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
    try:
        data = rospy.get_param('/camera_calibration')
        K = np.array(data['camera_matrix']['data']).reshape(3,3)
        D = np.array(data['dist_coeffs']['data'])
        return K, D
    except KeyError:
        rospy.logerr("Camera calibration not found on param server.")
        rospy.signal_shutdown("Missing calibration")
        return None, None

def classify_emotion(frame):

    return 'neutral'

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        self.bridge = CvBridge()
        self.K, self.D = load_camera_calib()

        # publishers
        self.pose_pub    = rospy.Publisher('/detected_card_pose', PoseStamped, queue_size=1)
        self.text_pub    = rospy.Publisher('/detected_card_text', String, queue_size=1)
        self.emotion_pub = rospy.Publisher('/player_emotion', String, queue_size=1)

        # static TF
        tf_broad = tf2_ros.StaticTransformBroadcaster()
        static_tf = TransformStamped()
        static_tf.header.stamp    = rospy.Time.now()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id  = 'camera_link'
        static_tf.transform.translation.x = 0.5
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 1.2
        static_tf.transform.rotation.w    = 1.0
        tf_broad.sendTransform(static_tf)
        rospy.loginfo("Published static TF base_link→camera_link")

        # A5 card corners in camera frame
        w, h = 0.74, 0.105
        self.obj_pts = np.array([
            [-w/2, -h/2, 0],
            [ w/2, -h/2, 0],
            [ w/2,  h/2, 0],
            [-w/2,  h/2, 0]
        ], dtype=np.float64)

        topic = rospy.get_param('~image_topic', '/camera/image_raw')
        rospy.Subscriber(topic, Image, self.callback, queue_size=1)
        rospy.loginfo(f"Subscribed to image topic: {topic}")
        rospy.spin()

    def callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # Emotion
        emotion = classify_emotion(frame)
        self.emotion_pub.publish(String(data=emotion))
        rospy.loginfo(f"Emotion: {emotion}")

        # Detect all QR codes
        codes = decode(frame)
        if not codes:
            return

        for qr in codes:
            pts = [(p.x, p.y) for p in qr.polygon]
            if len(pts) != 4:
                rospy.logwarn("Skipping QR with %d points", len(pts))
                continue
            img_pts = np.array(pts, dtype=np.float64)

            # pose estimation
            try:
                ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_pts, self.K, self.D)
            except Exception as e:
                rospy.logerr("solvePnP error: %s", e)
                continue
            if not ok:
                rospy.logwarn("solvePnP returned not ok")
                continue

            # publish PoseStamped
            ps = PoseStamped()
            ps.header.stamp    = rospy.Time.now()
            ps.header.frame_id = 'camera_link'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = tvec.flatten().tolist()

            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3,:3] = R
            q = tf_trans.quaternion_from_matrix(T)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            self.pose_pub.publish(ps)

            # publish card text
            text = qr.data.decode('utf-8')
            self.text_pub.publish(String(data=text))
            rospy.loginfo("Detected card: %s", text)
        # end for

if __name__ == '__main__':
    try:
        VisionNode()
    except rospy.ROSInterruptException:
        pass
