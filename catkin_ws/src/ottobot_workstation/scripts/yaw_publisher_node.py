#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ottobot_workstation.msg import YawFromQuaternion
# from scipy.spatial.transform import Rotation
import tf
from math import degrees, radians

def yaw_from_msg(orientation):
    euler_angle = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return degrees(euler_angle[-1])

class YawPublisher:
    def __init__(self):
        self.imu_subscriber = rospy.Subscriber("imu",  Imu, self.imu_callback)
        self.controller_subscriber = rospy.Subscriber("ottobot_diff_drive_controller/odom",  Odometry, self.controller_callback)
        self.filtered_subscriber = rospy.Subscriber("odometry/filtered",  Odometry, self.filtered_callback)
        self.publisher = rospy.Publisher("yaw", YawFromQuaternion, queue_size=10)

        self.yaw_imu = 0
        self.yaw_filtered = 0
        self.yaw_controller = 0

    def imu_callback(self, msg_data):
        self.yaw_imu = yaw_from_msg(msg_data.orientation)
    
    def controller_callback(self, msg_data):
        self.yaw_controller = yaw_from_msg(msg_data.pose.pose.orientation)

    def filtered_callback(self, msg_data):
        self.yaw_filtered = yaw_from_msg(msg_data.pose.pose.orientation)

    def publish(self):
        msg = YawFromQuaternion()
        msg.header.stamp = rospy.get_rostime()
        msg.imu_yaw = self.yaw_imu
        msg.controller_odom_yaw = self.yaw_controller
        msg.filtered_odom_yaw = self.yaw_filtered
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node("yaw_publisher_node", anonymous=True)
    yaw_publisher = YawPublisher()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        yaw_publisher.publish()
        r.sleep()
    