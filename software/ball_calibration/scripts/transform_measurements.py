#!/usr/bin/env python3
"""
Summary: Transforms camera measurements to world coordinates so 
don't need to log the transformations 
Author: Daniel Martin
"""
import rospy
import tf
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

listener = None

def measurement_callback(msg, args):
    transform_pub = args

    try:
        if transform_pub.get_num_connections():
            listener.waitForTransform(msg.header.frame_id, '/world', rospy.Time.now(), rospy.Duration(5))
            ps = listener.transformPose('/world', PoseStamped(msg.header, msg.pose.pose))
            transform_pub.publish(ps)
    except (tf.LookupException, tf.ConnectivityException):
        print(f"[Error] Couldn't find {msg.header.frame_id} to world!")


def main():
    rospy.init_node('transform_ball_node')

    global listener
    listener = tf.TransformListener()

    subs, pubs = dict(), dict()
    for i in range(6):
        topic_name = f"/ball_{i+1}_pose"
        transform_name = topic_name + "_transformed"
        pubs[i] = transform_pub = rospy.Publisher(transform_name,
                                                    PoseStamped, queue_size=1)
        subs[i] = rospy.Subscriber(topic_name, PoseWithCovarianceStamped, measurement_callback,
                                                    transform_pub, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
