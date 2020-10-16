#!/usr/bin/env python
import rospy
from ottobot_sense.srv import UpdateImuOffsets, UpdateImuOffsetsRequest

class ImuCalibrationCommander:
    def __init__(self):
        self.request = UpdateImuOffsetsRequest()
        self.load_offsets()

    def load_offsets(self):
         # Get parameters from provate namespace
        offsets = rospy.get_param('~imu_initial_offsets')
        # Accelerometer
        self.request.offsets.accelerometer.x = offsets.accel_offset_x
        self.request.offsets.accelerometer.y = offsets.accel_offset_y
        self.request.offsets.accelerometer.z = offsets.accel_offset_z
        self.request.offsets.accelerometer_radius = offsets.accel_radius
        # Gyroscope
        self.request.offsets.gyroscope.x = offsets.gyro_offset_x
        self.request.offsets.gyroscope.y = offsets.gyro_offset_y
        self.request.offsets.gyroscope.z = offsets.gyro_offset_z
        # Magnetometer
        self.request.offsets.magnetometer.x = offsets.mag_offset_x
        self.request.offsets.magnetometer.y = offsets.mag_offset_y
        self.request.offsets.magnetometer.z = offsets.mag_offset_z
        self.request.offsets.magnetometer_radius = offsets.mag_radius
        
    def update_client(self):
        # Wait for service to be advertised
        rospy.wait_for_service("set_imu_offsets")

        try:
            update_call = rospy.ServiceProxy("set_imu_offsets", UpdateImuOffsets)
            imu_response = update_call(self.request)
            return imu_response  # True/False
        except rospy.ServiceException as e:
            rospy.logerr("Imu offset update service call failed: {}".format(e))


if __name__ == '__main__':
    rospy.init_node("imu_calibration_commander", anonymous=True)
    imu_calibration_commander = ImuCalibrationCommander()
    imu_calibration_commander.update_client()
    