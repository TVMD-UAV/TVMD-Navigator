"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget, ManualControl, PositionTarget 
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import math
import sys
from select import select
from tasks import tasks_handler

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class Commander:
    def __init__(self) -> None:
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.local_pose_cb)

        # self.local_attitude_pub = rospy.Publisher("mavros/setpoint_raw/target_attitude", AttitudeTarget, queue_size=1)
        self.manual_pub = rospy.Publisher("mavros/manual_control/send", ManualControl, queue_size=1)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pos_raw_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.local_attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        self.old_std_settings = termios.tcgetattr(sys.stdin)

        self.state = State()
        self.local_pose = PoseStamped()
        self.takeoff_pose = np.zeros(3)
        self.terminate_pos = np.zeros(3)

        self.tasks = tasks_handler.TaskHandler()
        self.tasks.set_default()

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()

        rospy.loginfo("PX4 Connected!")
        self._send_dummy()
        rospy.loginfo("Dummy Finished")

        self._last_mode_req = rospy.Time.now()
        self._last_arm_req = rospy.Time.now()

        self.target_mode = "MANUAL"
        self.target_arm = False
        self.should_run = False
        self.start_time = 0
        self.count = 0

    def state_cb(self, msg):
        self.state = msg

    def local_pose_cb(self, msg):
        self.local_pose = msg


    def _send_dummy(self) -> None:
        # Send a few setpoints before starting
        for i in range(10):
            if(rospy.is_shutdown()):
                break

            self.tasks.pose_target_sp.header = rospy.Header()
            self.tasks.pose_target_sp.header.stamp = rospy.Time.now()
            self.pos_raw_pub.publish(self.tasks.pose_target_sp)
            self.manual_pub.publish(self.tasks.manual_sp)
            self.rate.sleep()

    def _try_set_mode(self, mode: str) -> None:
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = mode

        if(self.state.mode != mode and (rospy.Time.now() - self._last_mode_req) > rospy.Duration(1.0)):
            if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo(f"{mode} enabled")
            self._last_mode_req = rospy.Time.now()

    def _try_set_arm(self, arm: bool) -> None:
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = arm

        if(self.state.armed != arm and (rospy.Time.now() - self._last_arm_req) > rospy.Duration(0.5)):
            if(self.arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle {}".format("armed" if arm else "disarmed"))
                # self.takeoff_pose = self.local_pose.position
                self.takeoff_pose[0] = self.local_pose.pose.position.x
                self.takeoff_pose[1] = self.local_pose.pose.position.y
                self.takeoff_pose[2] = self.local_pose.pose.position.z
                rospy.loginfo("Home position set: {}".format(self.takeoff_pose))
            self._last_arm_req = rospy.Time.now()

    def print_help(self) -> None:
        print('''
            Usage:
            - a: arming
            - d: disarming 
            - o: switch to offboard mode
            - m: switch to manual mode
            - r: run the task
            - s: stop the task
            - [esc]: exit
            ''')

    def _handle_user_cmd(self) -> bool:        
        # https://stackoverflow.com/questions/2408560/non-blocking-console-input
        # https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
        rlist, _, _ = select([sys.stdin], [], [], 0)
        if rlist:  
            c = sys.stdin.read(1)
            if c == '\x1b':         # x1b is ESC
                # return true if it should stop
                return True
            elif c == 'a': 
                self.target_arm = True
            elif c == 'd':
                self.target_arm = False 
            elif c == 'o':
                self.target_mode = 'OFFBOARD'
            elif c == 'm':
                self.target_mode = 'MANUAL'
            elif c == 'r':
                self.should_run = True
                self.start_time = rospy.Time.now()
            elif c == 's':
                self.should_run = False
                self.terminate_pos[0] = self.local_pose.pose.position.x
                self.terminate_pos[1] = self.local_pose.pose.position.y
                self.terminate_pos[2] = self.local_pose.pose.position.z
            else:
                rospy.logwarn(f"Unknown command: {c}")
        return False
    
    """
    Using setpoint_raw to send position target
    """
    def update_task4(self, t):
        # https://github.com/mavlink/mavros/issues/471#issuecomment-172424122
        self.pose_target_sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pose_target_sp.type_mask = PositionTarget.IGNORE_YAW
        self.pose.header.frame_id = "odom_ned"
        
        # Initialization / default values
        self.pose_target_sp.position.x = self.takeoff_pose[0]
        self.pose_target_sp.position.y = self.takeoff_pose[1]
        self.pose_target_sp.position.z = self.takeoff_pose[2]

        self.pose_target_sp.velocity.x = 0
        self.pose_target_sp.velocity.y = 0
        self.pose_target_sp.velocity.z = 0

        self.pose_target_sp.acceleration_or_force.x = 0
        self.pose_target_sp.acceleration_or_force.y = 0
        self.pose_target_sp.acceleration_or_force.z = 0

        self.manual_sp.x = 0
        self.manual_sp.y = 0
        self.manual_sp.r = 0

        up_time = 2
        hover_time = 10
        land_time = 2
        shutdown_time = 2

        # Ascending param
        h = 1.5
        tau = up_time

        # Tilting param
        T_tilt = 6
        h_tilt = math.pi / 4
        # h_tilt = math.pi / 16

        t1 = up_time
        t2 = t1 + hover_time
        t3 = t2 + land_time
        t4 = t3 + shutdown_time

        # positions in NWU
        if t < t1:  # Ascending
            self.pose_target_sp.position.z = self.takeoff_pose[2] + (
                0.5 * h * (1 - math.cos(math.pi * (t - 0) / tau)))
            self.pose_target_sp.velocity.z = 0.5 * h* math.pi / tau * math.sin(math.pi * (t - 0) / tau)
            self.pose_target_sp.acceleration_or_force.z = 0.5 * h * (math.pi**2) / (tau**2) * math.cos(math.pi * (t - 0) / tau)
        elif t > t1 and t < t2: # Hover
            self.pose_target_sp.position.z = self.takeoff_pose[2] + h

            t_m = (t1 + t2) / 2
            if t > t_m - T_tilt / 2 and t < t_m + T_tilt / 2:
                # Scale for manual control: 1000
                self.manual_sp.y = 1000 * h_tilt * (1 + math.cos(2 * math.pi / T_tilt * (t - t_m)))
        elif t > t2 and t < t3: # Descending
            self.pose_target_sp.position.z = self.takeoff_pose[2] + h - (
                0.5 * h * (1 - math.cos(math.pi * (t - t2) / tau)))
            self.pose_target_sp.velocity.z = -0.5 * h* math.pi / tau * math.sin(math.pi * (t - t2) / tau)
            self.pose_target_sp.acceleration_or_force.z = -0.5 * h * (math.pi**2) / (tau**2) * math.cos(math.pi * (t - t2) / tau)
        elif t > t3 and t < t4: 
            self.pose_target_sp.position.z = self.takeoff_pose[2] - (t - t3)
        elif t > t4:
            self.pose_target_sp.position.z = -10
            self.should_run = False

    """
    Using setpoint_raw to send position target
    """
    def update_task5(self, t):
        # https://github.com/mavlink/mavros/issues/471#issuecomment-172424122
        self.pose_target_sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pose_target_sp.type_mask = PositionTarget.IGNORE_YAW
        self.pose.header.frame_id = "odom_ned"
        
        # Initialization / default values
        self.pose_target_sp.position.x = self.takeoff_pose[0]
        self.pose_target_sp.position.y = self.takeoff_pose[1]
        self.pose_target_sp.position.z = self.takeoff_pose[2]

        self.pose_target_sp.velocity.x = 0
        self.pose_target_sp.velocity.y = 0
        self.pose_target_sp.velocity.z = 0

        self.manual_sp.x = 0
        self.manual_sp.y = 0
        self.manual_sp.r = 0

        up_time = 2
        hover_time = 12
        land_time = 2
        shutdown_time = 2

        # Ascending param
        h = 1.5
        tau = up_time

        # Tilting param
        T_tilt = 2 * math.pi
        h_tilt = math.pi / 16
        l = 1

        t1 = up_time
        t2 = t1 + hover_time
        t3 = t2 + land_time
        t4 = t3 + shutdown_time

        # positions in NWU
        if t < t1:  # Ascending
            self.pose_target_sp.position.z = self.takeoff_pose[2] + (
                0.5 * h * (1 - math.cos(math.pi * (t - 0) / tau)))
            self.pose_target_sp.velocity.z = 0.5 * h* math.pi / tau * math.sin(math.pi * (t - 0) / tau)
        elif t > t1 and t < t2: # Hover
            self.pose_target_sp.position.z = self.takeoff_pose[2] + h

            t_m = (t1 + t2) / 2
            if t > t_m - T_tilt / 2 and t < t_m + T_tilt / 2:
                idx = 2 * math.pi / T_tilt * (t - t_m)
                self.pose_target_sp.position.x = l * math.cos(idx)
                self.pose_target_sp.position.y = l * math.sin(2 * idx)
                self.pose_target_sp.velocity.x = -2 * math.pi * l / T_tilt * math.sin(idx)
                self.pose_target_sp.velocity.y =  4 * math.pi * l / T_tilt * math.cos(2 * idx)

                # Scale for manual control: 1000
                # self.manual_sp.x = 1000 * h_tilt * (1 + math.cos(2 * math.pi / T_tilt * (t - t_m)))
        elif t > t2 and t < t3: # Descending
            self.pose_target_sp.position.z = self.takeoff_pose[2] + h - (
                0.5 * h * (1 - math.cos(math.pi * (t - t2) / tau)))
            self.pose_target_sp.velocity.z = -0.5 * h* math.pi / tau * math.sin(math.pi * (t - t2) / tau)
        elif t > t3 and t < t4: 
            self.pose_target_sp.position.z = self.takeoff_pose[2] - (t - t3)
        elif t > t4:
            self.pose_target_sp.position.z = -10
            self.should_run = False

    def pose_pretty_print(self) -> str:
        pos = self.tasks.pose_target_sp.position
        vel = self.tasks.pose_target_sp.velocity
        man = self.tasks.manual_sp
        return "p = [{:5.3f}, {:5.3f}, {:5.3f}], v = [{:5.3f}, {:5.3f}, {:5.3f}], q = [{:5.3f}, {:5.3f}, {:5.3f}]".format(
            pos.x, pos.y, pos.z, vel.x, vel.y, vel.z,
            man.x/1000, man.y/1000, man.r/1000)

    def main_loop(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            rospy.loginfo("Commander enters main loop!")
            while(not rospy.is_shutdown()):
                if (self._handle_user_cmd()):
                    break
                
                self._try_set_mode(self.target_mode)
                self._try_set_arm(self.target_arm)
                self.tasks.set_default()
                self.tasks.pose_target_sp.position.x = self.takeoff_pose[0]
                self.tasks.pose_target_sp.position.y = self.takeoff_pose[1]
                self.tasks.pose_target_sp.position.z = self.takeoff_pose[2]

                if (self.should_run):
                    t = rospy.Time.now() - self.start_time
                    self.should_run = self.tasks.task_tilting(t.to_sec(), self.takeoff_pose, self.should_run)
                    # self.should_run = self.tasks.task_8shape(t.to_sec(), self.takeoff_pose, self.should_run)
                    print("[{:7.2f}] {}".format(t.to_sec(), self.pose_pretty_print()))

                self.pos_raw_pub.publish(self.tasks.pose_target_sp)
                self.manual_pub.publish(self.tasks.manual_sp)
                self.local_attitude_pub.publish(self.tasks.attitude_target_sp)

                self.count += 1
                self.rate.sleep()

        except Exception as e:
            rospy.logerr(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_std_settings)
            rospy.loginfo("Leaving...")


def main():
    rospy.init_node("navigator_commander_node")
    commander = Commander()
    commander.print_help()
    commander.main_loop()
    
if __name__ == "__main__":
    main()
