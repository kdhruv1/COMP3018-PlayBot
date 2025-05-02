#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize
from tf.transformations import quaternion_from_euler
import sys

class GestureNode:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('gesture_node')
        self.arm = MoveGroupCommander("arm")
        self.arm.set_planning_time(5.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/detected_card_pose', PoseStamped, self.card_callback)
        rospy.loginfo("Gesture node ready.")
        rospy.spin()

    def card_callback(self, msg):
        try:
            # Transform pose to base_link frame
            target_pose = self.tf_buffer.transform(msg, 'base_link', rospy.Duration(1.0))
            pose_goal = self.arm.get_current_pose().pose
            pose_goal.position = target_pose.pose.position
            pose_goal.orientation = target_pose.pose.orientation  # Or set your own

            rospy.loginfo(f"Pointing to card at {pose_goal.position}")
            self.arm.set_pose_target(pose_goal)
            self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr(f"TF or motion error: {e}")

if __name__ == '__main__':
    try:
        GestureNode()
    except rospy.ROSInterruptException:
        pass
