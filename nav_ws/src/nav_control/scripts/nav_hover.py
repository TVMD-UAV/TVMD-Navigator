"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class Commander:
    def __init__(self) -> None:
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.local_pose_cb)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

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
        self.pose = PoseStamped()

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()

        rospy.loginfo("PX4 Connected!")
        self._send_dummy()

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

            self.local_pos_pub.publish(self.pose)
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
            else:
                rospy.logwarn(f"Unknown command: {c}")
        return False
    
    def update_task(self, t):
        self.pose.header.frame_id = "odom_ned"
        self.pose.pose.position.x = self.takeoff_pose[0]
        self.pose.pose.position.y = self.takeoff_pose[1]
        self.pose.pose.position.z = self.takeoff_pose[2]

        up_time = 2
        hover_time = 6
        land_time = 2
        shutdown_time = 2

        t1 = up_time
        t2 = t1 + hover_time
        t3 = t2 + land_time
        t4 = t3 + shutdown_time

        if t < t1:
            self.pose.pose.position.z = self.takeoff_pose[2] + 0.5 * t
        elif t > t1 and t < t2:
            self.pose.pose.position.z = self.takeoff_pose[2] + 0.5 * up_time
        elif t > t2 and t < t3:
            self.pose.pose.position.z = self.takeoff_pose[2] + 0.5 * up_time - 0.5 * (t - t2)
        elif t > t3 and t < t4: 
            self.pose.pose.position.z = self.takeoff_pose[2] - (t - t3)
        elif t > t4:
            self.pose.pose.position.z = -10
            self.should_run = False

    def pose_pretty_print(self) -> str:
        pose = self.pose.pose
        return "p = [{:5.3f}, {:5.3f}, {:5.3f}], q = [{:5.3f}, {:5.3f}, {:5.3f}, {:5.3f}]".format(
            pose.position.x, pose.position.y, pose.position.z, 
            pose.orientation.w, pose.orientation.w, pose.orientation.w, pose.orientation.w
        )

    def main_loop(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            rospy.loginfo("Commander enters main loop!")
            while(not rospy.is_shutdown()):
                if (self._handle_user_cmd()):
                    break
                self.pose.header = rospy.Header()
                
                self._try_set_mode(self.target_mode)
                self._try_set_arm(self.target_arm)

                if (self.should_run):
                    t = rospy.Time.now() - self.start_time
                    print("[{:7.2f}] {}".format(t.to_sec(), self.pose_pretty_print()))
                    self.update_task(t.to_sec())

                self.local_pos_pub.publish(self.pose)
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
