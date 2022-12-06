import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import rospy
import math
import matplotlib.pyplot as plt
import tf2_ros


if __name__ == "__main__":
    rospy.init_node('record_speed')

    listener = tf.TransformListener()


    poll_time = 20
    print("Poll data for", poll_time, "seconds")
    rate = rospy.Rate(100)
    start = rospy.Time.now()

    speeds = []
    angular_speeds = []
    times = []

    #listener.waitForTransform("/wam/racquet_hitpoint_link", "/wam/base_link", rospy.Time(0), rospy.Duration(0.3))
    while rospy.Time.now() - start < rospy.Duration(poll_time):
        try:
            # listener.waitForTransform("/wam/racquet_hitpoint_link", "/wam/base_link", rospy.Time(0), rospy.Duration(0.3))
            # tw = listener.lookupTwist("/wam/racquet_hitpoint_link", "/wam/base_link", rospy.Time(0), rospy.Duration(0.01))
            # listener.waitForTransform("/wam/racquet_hitpoint_link", "/world", rospy.Time(0), rospy.Duration(0.3))
            # tw = listener.lookupTwist("/wam/racquet_hitpoint_link", "/world", rospy.Time(0), rospy.Duration(0.01))
            listener.waitForTransform("/world", "/wam/racquet_hitpoint_link", rospy.Time(0), rospy.Duration(0.3))
            tw = listener.lookupTwist("/world", "/wam/racquet_hitpoint_link", rospy.Time(0), rospy.Duration(0.01))

            # tw = listener.lookupTwistFull("/wam/racquet_hitpoint_link", "/world", "/world", (0, 0, 0), "/wam/racquet_hitpoint_link", rospy.Time(0), rospy.Duration(0.01))
            # tw = listener.lookupTwistFull("/wam/racquet_hitpoint_link", "/wam/base_link", "/wam/base_link", (0, 0, 0), "/wam/base_link", rospy.Time(0), rospy.Duration(0.01))

            twist = Twist(Vector3(tw[0][0], tw[0][1], tw[0][2]),  Vector3(tw[1][0], tw[1][1], tw[1][2]))
            speed = math.sqrt(tw[0][0]**2 + tw[0][1]**2 + tw[0][2]**2)
            angular_speed = math.sqrt(tw[1][0]**2 + tw[1][1]**2 + tw[1][2]**2)
            speeds.append(speed)
            angular_speeds.append(angular_speed)
            times.append((rospy.Time.now() - start).to_sec())
        except (tf.LookupException, tf.ConnectivityException):
            print("[Warn] TF Problem! Exiting...")
            break
        # rate.sleep()

    if len(speeds) > 0:
        plt.title('Racket Speeds')
        plt.plot(times, speeds)
        plt.plot(times, angular_speeds)
        plt.legend(["Linear velocity magnitude", "Angular velocity magnitude"])
        plt.grid()
        plt.show()
    else:
        print("Tf Problems")
