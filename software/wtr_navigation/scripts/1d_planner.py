import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from wtr_navigation.msg import WheelInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from numpy import clip, sign

CONTROL_LOOP_RATE = 250
MAX_ACC = 1.2
MAX_DEACC = 1.2 #2.2
MAX_VEL = 2.5
dT = 1/CONTROL_LOOP_RATE
WHEEL_RADIUS = 0.319

current_position = 0 #None
desired_position = 0 #None
current_velocity = None
pub_cmd_vel = None
vis_pub = None
pub_msg = WheelInfo()

commanded_vel = 0

def robot_odom_callback(msg):
    global current_position, current_velocity
    current_position = msg.pose.pose.position.x
    current_velocity = msg.twist.twist.linear.x

def desired_position_callback(msg):
    global desired_position, commanded_vel
    desired_position = msg.data

    vis_desired_pos = PointStamped(header=Header(frame_id="world"), point=Point(desired_position, 0., 0.))
    vis_pub.publish(vis_desired_pos)

    commanded_vel = 0

    # print("==============")

def update_wheel_velocity(event):
    global pub_msg, commanded_vel

    # vel = current_velocity

    # already at desired position. Stop moving
    if abs(desired_position - current_position) < 3e-2:
        pub_msg.left = 0
        pub_msg.right = 0

        pub_cmd_vel.publish(pub_msg)

        return

    direction = sign(desired_position - current_position)

    # Check if we need to de-accelerate [ based on V^2 = V_0^2 + 2A(x-x_0) ]
    # if current_velocity**2 /(2*MAX_DEACC) >= abs(desired_position-current_position):
    if commanded_vel**2 /(2*MAX_DEACC) >= abs(desired_position-current_position):
        # vel += -sign(vel) * MAX_DEACC * dT
        commanded_vel += -direction * MAX_DEACC * dT
    else:
        # We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
        # vel = clip(vel + sign(desired_position - current_position) * MAX_ACC * dT, -MAX_VEL, MAX_VEL)
        commanded_vel = clip(commanded_vel + (direction * MAX_ACC * dT), -MAX_VEL, MAX_VEL)

    # convert linear velocity to left and right radial velocities
    # pub_msg.left = vel/WHEEL_RADIUS
    # pub_msg.right = vel/WHEEL_RADIUS
    pub_msg.left = commanded_vel/WHEEL_RADIUS
    pub_msg.right = commanded_vel/WHEEL_RADIUS

    print(pub_msg)
    pub_cmd_vel.publish(pub_msg)

    return
        

def main():
    global pub_cmd_vel, vis_pub

    rospy.init_node("pid_wheels_script", anonymous=True)

    # Get Robot Position
    # rospy.Subscriber("/wcodometry_global", Odometry,
    #                  robot_odom_callback, queue_size=1)
    rospy.Subscriber("/wcodometry_local", Odometry,
                     robot_odom_callback, queue_size=1)

    # Get Robot's desired position
    rospy.Subscriber("/wheelchair_desired_position", Float64,
                     desired_position_callback, queue_size=1)

    # publisher that takes the velocity commanded from pid cpntroller and converts it to the right message for teensy
    pub_cmd_vel = rospy.Publisher('/wheel_cmd_vel', WheelInfo, queue_size=1)

    # publish a visualization message to compare performance
    vis_pub = rospy.Publisher("/vis_start_move", PointStamped, queue_size=1)


    rospy.Timer(rospy.Duration(1.0 / CONTROL_LOOP_RATE), update_wheel_velocity)

    rospy.spin()

if __name__ == '__main__':
    main()