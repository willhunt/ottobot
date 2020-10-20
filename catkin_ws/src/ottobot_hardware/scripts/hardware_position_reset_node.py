#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class JointPositionResetter:
    def __init__(self):
        self.publisher = rospy.Publisher("hardware/reset_joint_positions", Bool, queue_size=1)

    def reset_joint_positions(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        rospy.loginfo("Hardware Joint Resetter: Wheel positions reset to zero.")

if __name__ == '__main__':
    rospy.init_node("hardware_position_reset_node", anonymous=True)
    resetter = JointPositionResetter()
    resetter.reset_joint_positions()
    