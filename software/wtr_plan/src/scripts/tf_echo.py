#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('tf_echo')

    pub_trans = rospy.Publisher('base_link_trans_hitpoint_tf', Float32MultiArray, queue_size = 1)
    pub_rot = rospy.Publisher('base_link_rot_hitpoint_tf', Float32MultiArray, queue_size = 1)
    
    listener = tf.TransformListener()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('wam/base_link', 'wam/racquet_hitpoint_link', rospy.Time(0))

            pub_data_trans = Float32MultiArray()
            pub_data_trans.data = trans
            pub_trans.publish(pub_data_trans)

            (roll, pitch, yaw) = euler_from_quaternion (rot)
            pub_data_rot = Float32MultiArray()
            pub_data_rot.data = [roll, pitch, yaw]
            pub_rot.publish(pub_data_rot)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print("----------")
        # print(trans)
        # print(rot)
        # print("----------")

        

        rate.sleep()