import rospy
from sensor_msgs.msg import Imu
import threespace_api as ts_api
import time

com_port = "/dev/ttyACM3"
device = ts_api.TSUSBSensor(com_port=com_port)

# Getting and printing the firmware and hardware version
firmware_version = device.getFirmwareVersionString()
print(f'Firmware Version:  {firmware_version}')

hardware_version = device.getHardwareVersionString()
print(f'Hardware Version:  {hardware_version}')

# Auto calibrating the gyroscope
success = device.beginGyroscopeAutoCalibration()
if not success:
    raise Exception('Gyroscope Calibration Failed!!!!')

# Setting filter mode to Q-COMP
success = device.setFilterMode(2)
if not success:
    raise Exception('Filter mode could not be set to Q-COMP!!!!')

# Setting and Checking current axis directions
success = device.setAxisDirections(3)
if not success:
    raise Exception('Could not set the right axis directions!!!!')
axis_direction = device.getAxisDirections()
print(f"Current Axis Directions: {axis_direction}")

# Setting and Checking Euler decomposition
success = device.setEulerAngleDecompositionOrder(3)
if not success:
    raise Exception('Could not set the Euler angle decomposition same as theaxis directions on device!!!!')
euler_decomposition = device.getEulerAngleDecompositionOrder()
print(f"Euler Decomposition: {euler_decomposition}")

# Checking if compass enabled
compass_enabled = device.getCompassEnabledState()
print(f"Compass Enabled State : {compass_enabled}")


def imu_stream_publish():
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(500) # 10hz
    imu_msg = Imu()

    while not rospy.is_shutdown():
        if device.stream_last_data != None:
            parsed_val = device.stream_last_data[1]
            imu_msg.header.stamp    = rospy.get_rostime()
            imu_msg.header.frame_id = "base_footprint"
            imu_msg.orientation.x = parsed_val[0]
            imu_msg.orientation.y = parsed_val[1]
            imu_msg.orientation.z = parsed_val[2]
            imu_msg.orientation.w = parsed_val[3]
            imu_msg.angular_velocity.x = parsed_val[4]
            imu_msg.angular_velocity.y = parsed_val[5]
            imu_msg.angular_velocity.z = parsed_val[6]
            imu_msg.linear_acceleration.x = parsed_val[7]
            imu_msg.linear_acceleration.y = parsed_val[8]
            imu_msg.linear_acceleration.z = parsed_val[9]
            pub.publish(imu_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Setting streaming time of 2 ms i.e. 500 Hz
        device.setStreamingTiming(2000, 0xFFFFFFFF, 0)
        device.setStreamingSlots(slot0='getUntaredOrientationAsQuaternion',
                         slot1='getCorrectedGyroRate',
                         slot2='getCorrectedLinearAccelerationInGlobalSpace')
        device.startStreaming()
        imu_stream_publish()
    except rospy.ROSInterruptException:
        device.stopStreaming()


# start_time = time.perf_counter()
# while time.perf_counter() - start_time < 10: 
#     if device.stream_last_data != None:
#         print(device.stream_last_data[1])
#         print(type(device.stream_last_data)) 
#         print("=======================================\n")
#         time.sleep(0.002)



      




