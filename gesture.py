import rospy
import sys
import tf2_ros
import tf2_geometry_msgs  # for do_transform_pose
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize

class GestureNode:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('gesture_node')

        # MoveIt setup
        self.arm = MoveGroupCommander("manipulator")
        self.arm.set_planning_time(5.0)

        # TF for transforming camera_link to base_link
        self.tf_buf  = tf2_ros.Buffer()
        self.tf_lisn = tf2_ros.TransformListener(self.tf_buf)

        rospy.Subscriber('/detected_card_pose', PoseStamped, self.card_cb, queue_size=1)
        rospy.loginfo("Gesture node ready.")
        rospy.spin()

    def card_cb(self, ps_msg):
        try:
            # transform into base_link
            target = self.tf_buf.transform(ps_msg, 'base_link', rospy.Duration(1.0))
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = 'base_link'
            pose_goal.pose = target.pose

            rospy.loginfo("Pointing to card at (%.3f, %.3f, %.3f)",
                          pose_goal.pose.position.x,
                          pose_goal.pose.position.y,
                          pose_goal.pose.position.z)

            self.arm.set_pose_target(pose_goal)
            self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr("Gesture error: %s", e)

if __name__ == '__main__':
    try:
        GestureNode()
    except rospy.ROSInterruptException:
        pass
