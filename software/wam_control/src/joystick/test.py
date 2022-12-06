#/usr/bin/env python
"""
Requires:
* Xbox controller
* joy_node must be running (http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

..code:: bash
    
    rosparam set joy_node/dev "/dev/input/jsX"
    rosrun joy joy_node

* joint position controller running on the arm
"""
from typing import Union, List, Tuple
from collections import namedtuple
import enum
import numpy as np
import rospy
from sensor_msgs.msg import Joy, JointState
from wtr_navigation.msg import WheelInfo
from std_msgs.msg import Float64MultiArray


class MODE(enum.Enum):
    IDLE = 0        # doesn't provide any commands
    ACTIVE = 1      # accepting commands
    SWING_READY = 3 # swing ready state needed to enter swing
    SWING = 4    # can control the swing params

# wired setting
# class Xbox:
#     class Button(enum.Enum):
#         A = 0
#         B = 1
#         X = 2
#         Y = 3
#         LB = 4
#         RB = 5
#         BACK = 6
#         START = 7
#         POWER = 8

#     class Axes(enum.Enum):
#         LS_LR = 0
#         LS_UD = 1
#         LT = 2          # starts at 1 goes to one -1
#         RS_LR = 3
#         RS_UD = 4
#         RT = 5          # similar to LT range
#         CROSS_LR = 6    # discrete (-1,0,1)
#         CROSS_UD = 7    # discrete (-1,0,1)

# bluetooth setting
class Xbox:
    class Button(enum.Enum):
        A = 0
        B = 1
        X = 3
        Y = 4
        LB = 6
        RB = 7
        BACK = -1
        START = 11
        POWER = -1

    class Axes(enum.Enum):
        LS_LR = 0
        LS_UD = 1
        LT = 5          # starts at 1 goes to one -1
        RS_LR = 2
        RS_UD = 3
        RT = 4          # similar to LT range
        CROSS_LR = 6    # discrete (-1,0,1)
        CROSS_UD = 7    # discrete (-1,0,1)

class CatapultSwing:
    JOINT_NAMES = [
        'wam/base_yaw_joint',
        'wam/shoulder_pitch_joint',
        'wam/shoulder_yaw_joint',
        'wam/elbow_pitch_joint',
        'wam/wrist_yaw_joint',
        'wam/wrist_pitch_joint',
        'wam/palm_yaw_joint',
    ]
    HOME_POS = [0, -1.974, 0, 3.01, 0, 0, 0]

    # configured for forehand stroke (TODO include backhand stroke)
    SHOULDER_PITCH_RANGE = [-1.57, -0.4]
    DEFAULT_PITCH_POS = -1.2

    SWING_INIT_POS = [-1.115, DEFAULT_PITCH_POS, 1.57, 1.35, -1.57, 0.0, 0.0]
    SWING_MID_POS = [0.0, DEFAULT_PITCH_POS, 1.57, 0, -1.57, 0.0, 0.0]
    SWING_END_POS = [1.115, DEFAULT_PITCH_POS, 1.57, -1.35, -1.57, 0.0, 0.0]

    SWING_DURATION = 1
    CONTROL_RATE = 100

    JPOS_MIN = [-2.6, -1.98, -2.8, -0.9, -4.65, -1.57, -2.95]
    JPOS_MAX = [ 2.6,  1.98,  2.8,  3.1,  1.35,  1.57,  2.95]

    JVEL = [3.9, 6.3, 10.0, 10.0, 24.0, 19.0, 27.0]
    JACC = [6.82, 11.5, 17.0, 21.5, 84.0, 110.0, 100.0]


    def __init__(self):
        self.jn_idx = {jn: idx for idx, jn in enumerate(self.JOINT_NAMES)}
        self.jpos_profile = np.array(self.traj_profile())
        self.n_steps = len(self.jpos_profile)
        self.shoulder_pitch_change_threshold = 0.5 * self.JVEL[1] * (1 / self.CONTROL_RATE)
        self.shoulder_slope = (self.SHOULDER_PITCH_RANGE[1] - self.SHOULDER_PITCH_RANGE[0]) / 2
        self.init()
 

    def get_jpos(self, jstate: JointState):
        jpos = [0] * len(self.JOINT_NAMES)
        for jn, pos in zip(jstate.name, jstate.position):
            jpos[self.jn_idx[jn]] = pos
        return jpos


    def init(self):
        self.curr_shoulder_pitch = self.DEFAULT_PITCH_POS
        self.cmd_shoulder_pitch = self.DEFAULT_PITCH_POS
        self.jpos_profile[:,1] = self.curr_shoulder_pitch
        self.t = 0


    def update(self, cmd):
        # using only cmd[0] for shoulder pitch, considering using cmd[1] to control racket yaw
        if cmd[0]:
            self.cmd_shoulder_pitch -= 0.1
        elif cmd[1]:
            self.cmd_shoulder_pitch += 0.1
        self.cmd_shoulder_pitch = np.clip(self.cmd_shoulder_pitch, *self.SHOULDER_PITCH_RANGE)


    def step(self) -> bool:
        """return's True if swing ended"""
        if self.t == self.n_steps:
            return True, None
        pos = self.jpos_profile[self.t,:]
        self.t += 1
        self.curr_shoulder_pitch += np.clip(
            self.cmd_shoulder_pitch - self.curr_shoulder_pitch, 
            -self.shoulder_pitch_change_threshold, 
            self.shoulder_pitch_change_threshold
        )
        self.curr_shoulder_pitch = np.clip(self.curr_shoulder_pitch, *self.SHOULDER_PITCH_RANGE)
        pos[1] = self.curr_shoulder_pitch
        return False, pos


    def traj_profile(self):

        profiles = {
            'p_start': [],
            'time_end_ramp': [],
            'p_end_ramp': [],
            'time_peak': [],
            'time_start_deramp': [],
            'p_start_deramp': [],
            'time_end': [],
            'p_end': [],
            'acc': [],
            'vel': [],
        }
        
        longest_profile_idx = 0
        longest_profile_time = -1

        num_joints = len(self.JPOS_MIN)
        precision = 6

        for i in range(num_joints):
            start_pos = self.SWING_INIT_POS[i]
            mid_pos = self.SWING_MID_POS[i]
            end_pos = self.SWING_END_POS[i]
            delta_pos = end_pos - start_pos

            sign = 1 if delta_pos >= 0 else -1
            max_acc = sign * self.JACC[i]

            max_vel_init2mid = np.sqrt(2 * max_acc * (mid_pos - start_pos))
            max_vel_mid2end = np.sqrt(2 * max_acc * (end_pos - mid_pos))
            max_vel = sign * min(self.JVEL[i], max_vel_init2mid, max_vel_mid2end)

            time_end_ramp = max_vel / max_acc
            p_end_ramp = start_pos + 0.5 * max_acc * (time_end_ramp ** 2)
            p_start_deramp = end_pos - (max_vel ** 2) / (2 * max_acc)
            time_start_deramp = time_end_ramp + \
                (abs((p_start_deramp - p_end_ramp) / max_vel) if max_vel != 0 else 0)
            time_end = time_start_deramp + max_vel / max_acc
            time_peak = time_end_ramp + (time_end_ramp - time_start_deramp) / 2

            if time_end > longest_profile_time:
                longest_profile_idx = i
                longest_profile_time = time_end

            profiles['p_start'].append(start_pos)
            profiles['time_end_ramp'].append(np.round(time_end_ramp, precision))
            profiles['p_end_ramp'].append(p_end_ramp)
            profiles['p_start_deramp'].append(p_start_deramp)
            profiles['time_start_deramp'].append(np.round(time_start_deramp, precision))
            profiles['p_end'].append(end_pos)
            profiles['time_end'].append(np.round(time_end, precision))
            profiles['time_peak'].append(np.round(time_peak, precision))
            profiles['acc'].append(max_acc)
            profiles['vel'].append(max_vel)

        unique_times = set([])
        
        for i in range(num_joints):
            time_shift = profiles['time_peak'][longest_profile_idx] - profiles['time_peak'][i]
            unique_times.add(time_shift)
            unique_times.add(time_shift + profiles['time_end_ramp'][i])
            unique_times.add(time_shift + profiles['time_start_deramp'][i])
            unique_times.add(time_shift + profiles['time_end'][i])
        
        unique_times = list(unique_times)
        unique_times.sort()

        traj_profile = {}

        for idx, unique_time in enumerate(unique_times):
            traj_profile[idx] = {
                'pos': [],
                'vel': [],
                'acc': [],
            }
            for i in range(num_joints):
                adjusted_t = unique_time - (profiles['time_peak'][longest_profile_idx] - profiles['time_peak'][i])
                if adjusted_t < 0:
                    pos = profiles['p_start'][i]
                    vel = 0
                    acc = 0
                elif adjusted_t < profiles['time_end_ramp'][i]:
                    pos = profiles['p_start'][i] + 0.5 * profiles['acc'][i] * (adjusted_t ** 2)
                    vel = profiles['acc'][i] * adjusted_t
                    acc = profiles['acc'][i]
                elif adjusted_t < profiles['time_start_deramp'][i]:
                    pos = profiles['p_end_ramp'][i] + profiles['vel'][i] * (adjusted_t - profiles['time_end_ramp'][i])
                    vel = profiles['vel'][i]
                    acc = 0.0
                elif adjusted_t < profiles['time_end'][i]:
                    dt = adjusted_t - profiles['time_end'][i]
                    pos = profiles['p_end'][i] - (0.5 * profiles['acc'][i] * (dt ** 2))
                    vel = -profiles['acc'][i] * dt
                    acc = -profiles['acc'][i]
                else:
                    pos = profiles['p_end'][i]
                    vel = 0
                    acc = 0
                traj_profile[idx]['pos'].append(pos)
                traj_profile[idx]['vel'].append(vel)
                traj_profile[idx]['acc'].append(acc)
        
        jpos_profiles = []
        N = len(unique_times)
        for uid in range(len(unique_times) - 1):
            istart = unique_times[uid]
            iend = unique_times[uid + 1]
            traj_p = traj_profile[uid]
            times = np.linspace(istart, iend, int((iend - istart) * self.CONTROL_RATE), endpoint=False)
            for t in times:
                jpos_profiles.append([])
                for i in range(num_joints):
                    dt = t - istart
                    pos = traj_p['pos'][i] + traj_p['vel'][i] * dt + 0.5 * traj_p['acc'][i] * (dt ** 2)
                    jpos_profiles[-1].append(pos)

        return jpos_profiles


class WAMCommander:

    def __init__(self):
        self.pub = rospy.Publisher('/joint_position_controller/command', Float64MultiArray, queue_size=5)
    
    def exec(self, jpos: Union[List[List[float]], List[float]], rate=200):
        rate = rospy.Rate(rate)
        if type(jpos[0]) != list:
            msg = Float64MultiArray(data=jpos)
            self.pub.publish(msg)
        else:
            for pos in jpos:
                msg = Float64MultiArray(data=pos)
                self.pub.publish(msg)
                rate.sleep()


class WheelchairCommander:
    LINEAR_VEL_MAX = 1.0    # maximum linear velocity [m/s]
    ANGULAR_VEL_MAX = 1.0   # maximum angular velocity [rad/s]
    WHEEL_SEPARATION = 0.92 # [m]
    WHEEL_RADIUS = 0.3225   # [m]

    def __init__(self):
        # setting up publisher for publishing commanded wheel velocities
        self.cmd_wheel = rospy.Publisher('/wheel_cmd_vel', WheelInfo, queue_size=1)


    def convert_cmd(self, cmd: Tuple[float, float]):
        """perform log map from cmd to velocity spaces"""
        return (
            cmd[0] * abs(cmd[0]) * self.LINEAR_VEL_MAX,
            cmd[1] * abs(cmd[1]) * self.ANGULAR_VEL_MAX
        )


    def exec(self, cmd: Tuple[float, float]):
        """execture command in range (-1, 1) corresponding to throttle, steering"""
        throttle, steering = self.convert_cmd(cmd)
        
        # compute wheel velocities
        cmd_wheel_msg = WheelInfo()
        cmd_wheel_msg.left = (throttle + (steering * self.WHEEL_SEPARATION * 0.5)) / self.WHEEL_RADIUS
        cmd_wheel_msg.right = (throttle - (steering * self.WHEEL_SEPARATION * 0.5)) / self.WHEEL_RADIUS

        self.cmd_wheel.publish(cmd_wheel_msg)


class JoyStickController:
    """Makes swing joint configurable with joystick"""
    COMMAND_NAMES = ['throttle', 'steering', 'shoulder_pitch']
    Command = namedtuple('Command', COMMAND_NAMES, defaults=[0, 0, 1])

    # ---------------------------------------------------------------------------
    cmd = Command()
    mode = MODE.IDLE
    swing = CatapultSwing()
    wam_position: List[float] = None

    def __init__(self):
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.process_cmd, queue_size=1)
        self.jstate_subscriber = rospy.Subscriber('joint_states', JointState, self.process_jstate, queue_size=1)
        self.wam_commander = WAMCommander()
        self.wheelchair_commander = WheelchairCommander()

        rate = rospy.Rate(self.swing.CONTROL_RATE)
        while not rospy.is_shutdown():
            if self.mode == MODE.SWING:
                complete, jpos = self.swing.step()
                if not complete:
                    self.wam_commander.exec(jpos)
                else:
                    rospy.sleep(rospy.Duration(2))
                    self.reset(self.swing.SWING_INIT_POS, t_sec=3)
                    self.mode = MODE.SWING_READY
            rate.sleep()


    def reset(self, target_pos: List[float], t_sec=5):
        curr_pos = np.array(self.wam_position.copy())
        target_pos = np.array(target_pos)

        diff = target_pos - curr_pos
        control_rate = 200
        steps = t_sec * control_rate
        traj = []
        for t in range(control_rate):
            traj.append((curr_pos + (diff * (1 / steps / 2.0) * t)).tolist())
        for t in range(steps - int(control_rate // 2)):
            traj.append((curr_pos + (diff * (1 / steps) * (t + (control_rate // 2)))).tolist())
        self.wam_commander.exec(traj)


    def process_cmd(self, joy: Joy):
        if self.mode is MODE.SWING:
            # continue swing based on commands
            self.swing.update((
                joy.buttons[Xbox.Button.LB.value] * -1,
                joy.buttons[Xbox.Button.RB.value] * -1))
            return

        if joy.buttons[Xbox.Button.START.value]:
            if self.mode is MODE.IDLE:
                self.mode = MODE.ACTIVE
                rospy.loginfo(f'moving arm to home position = {self.swing.HOME_POS}')
                self.reset(self.swing.HOME_POS, 3)
                rospy.loginfo(f'toggled from {MODE.IDLE} to {MODE.ACTIVE}')
            else:
                self.mode = MODE.IDLE
                rospy.loginfo(f'moving arm to home position = {self.swing.HOME_POS}')
                self.reset(self.swing.HOME_POS, 3)
                rospy.loginfo(f'toggled from {self.mode} to {MODE.IDLE}')
            return

        if self.mode is MODE.IDLE:
            return

        self.wheelchair_commander.exec((
            joy.axes[Xbox.Axes.LS_UD.value],
            joy.axes[Xbox.Axes.RS_LR.value]))

        if joy.buttons[Xbox.Button.X.value]:
            if self.mode is MODE.ACTIVE:
                rospy.loginfo(f'moving arm to initial swing position = {self.swing.SWING_INIT_POS}')
                self.reset(self.swing.SWING_INIT_POS, 3)
                self.mode = MODE.SWING_READY
            elif self.mode is MODE.SWING_READY:
                rospy.loginfo(f'moving arm to home position = {self.swing.HOME_POS}')
                self.reset(self.swing.HOME_POS, 3)
                self.mode = MODE.ACTIVE

        if self.mode is MODE.SWING_READY and joy.buttons[Xbox.Button.Y.value]:
            self.mode = MODE.SWING
            self.wheelchair_commander.exec([0.0, 0.0])
            self.swing.init()


    def process_jstate(self, jstate: JointState):
        self.wam_position = self.swing.get_jpos(jstate)


if __name__ == '__main__':
    rospy.init_node('js_catapult_jposctrl')
    JoyStickController()

