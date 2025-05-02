#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg

class GestureNode:
    def __init__(self):
        # Initialize ROS and MoveIt
        moveit_commander.roscpp_initialize([])
        rospy.init_node('gesture_node', anonymous=True)

        # Subscribers
        self.last_card_pose = None
        rospy.Subscriber('/detected_card_pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/robot_blackjack_move', String, self.move_callback, queue_size=1)

        # MoveIt group for robot arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # Optional: named pose 'home'
        rospy.loginfo("Gesture node ready, waiting for commands...")
        rospy.spin()

    def pose_callback(self, msg: PoseStamped):
        # Store latest detected card pose
        self.last_card_pose = msg

    def move_callback(self, msg: String):
        cmd = msg.data.lower()
        if cmd == 'hit':
            rospy.loginfo("Gesture: pointing to next card to 'hit'.")
            if self.last_card_pose:
                self.gesture_towards(self.last_card_pose)
            else:
                rospy.logwarn("No card pose available to gesture to.")
        elif cmd == 'stand':
            rospy.loginfo("Gesture: no card selected (stand). Moving to home.")
            self.go_home()

    def gesture_towards(self, card_pose: PoseStamped):
        # Create a target pose slightly above the card
        target_pose = PoseStamped()
        target_pose.header.frame_id = card_pose.header.frame_id
        target_pose.pose.position.x = card_pose.pose.position.x
        target_pose.pose.position.y = card_pose.pose.position.y
        target_pose.pose.position.z = card_pose.pose.position.z + 0.15  # hover 15cm above
        # Orientation: point end-effector downwards
        # (Assumes tool frame oriented Z-outwards)
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(-3.1415/2, 0, 0)  # pitch down
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Plan and execute
        self.arm.set_pose_target(target_pose)
        plan = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        # Hold for a moment
        rospy.sleep(1.0)

        # Return to home
        self.go_home()

    def go_home(self):
        try:
            self.arm.set_named_target('home')
            self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception as e:
            rospy.logwarn(f"Failed to go to home pose: {e}")

if __name__ == '__main__':
    try:
        GestureNode()
    except rospy.ROSInterruptException:
        pass
