import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from wtr_navigation.msg import WheelInfo

pub_current_position = None
pub_cmd_vel = None

def robot_odom_callback(msg):
    current_position = msg.pose.pose.position.x

    pub_current_position.publish(current_position)

def set_wheel_speeds(msg):
    desired_speed = msg.data

    pub_msg = WheelInfo()

    pub_msg.left = desired_speed
    pub_msg.right = desired_speed

    pub_cmd_vel.publish(pub_msg)

def main():
    global pub_current_position, pub_cmd_vel

    rospy.init_node("pid_wheels_script", anonymous=True)

    # Get Robot Position
    # rospy.Subscriber("/wcodometry_global", Odometry,
    #                  robot_odom_callback, queue_size=1)
    rospy.Subscriber("/wcodometry_local", Odometry,
                     robot_odom_callback, queue_size=1)

    # Get Pid Commands
    rospy.Subscriber("/wheels_speed", Float64,
                     set_wheel_speeds, queue_size=1)

    # publisher that takes in odom message and publishes a single value for the pid controller (1D control)
    pub_current_position = rospy.Publisher('/wheelchair_current_position', Float64, queue_size=1)

    # publisher that takes the velocity commanded from pid cpntroller and converts it to the right message for teensy
    pub_cmd_vel = rospy.Publisher('/wheel_cmd_vel', WheelInfo, queue_size=1)
    
    

    rospy.spin()

if __name__ == '__main__':
    main()