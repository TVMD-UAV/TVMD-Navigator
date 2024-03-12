#! /usr/bin/env python
import numpy as np
import tf
from math import cos, sin, pi
import rospy
from mavros_msgs.msg import State, AttitudeTarget, ManualControl, PositionTarget 


class TaskHandler:
    def __init__(self):
        self.attitude_target_sp = AttitudeTarget()
        self.manual_sp = ManualControl()
        self.pose_target_sp = PositionTarget()
        self.count = 0

    def set_default(self):
        self.count += 1
        header = rospy.Header()
        header.seq = self.count
        header.stamp = rospy.Time.now()

        # https://github.com/mavlink/mavros/issues/471#issuecomment-172424122
        self.pose_target_sp.header.frame_id = "odom_ned"
        self.pose_target_sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pose_target_sp.type_mask = PositionTarget.IGNORE_YAW

        self.attitude_target_sp.type_mask = AttitudeTarget.IGNORE_THRUST + AttitudeTarget.IGNORE_ATTITUDE

        self.pose_target_sp.position.x = 0
        self.pose_target_sp.position.y = 0
        self.pose_target_sp.position.z = 0
        
        self.pose_target_sp.velocity.x = 0
        self.pose_target_sp.velocity.y = 0
        self.pose_target_sp.velocity.z = 0

        self.pose_target_sp.acceleration_or_force.x = 0
        self.pose_target_sp.acceleration_or_force.y = 0
        self.pose_target_sp.acceleration_or_force.z = 0

        self.attitude_target_sp.orientation.x = 0
        self.attitude_target_sp.orientation.y = 0
        self.attitude_target_sp.orientation.z = 0
        self.attitude_target_sp.orientation.w = 1

        self.attitude_target_sp.body_rate.x = 0
        self.attitude_target_sp.body_rate.y = 0
        self.attitude_target_sp.body_rate.z = 0

        self.manual_sp.x = 0
        self.manual_sp.y = 0
        self.manual_sp.r = 0
        self.manual_sp.z = 0    # Throttle
      
    """
    Using setpoint_raw to send position target
    """
    def task_tilting(self, t: float, takeoff_pose, should_run):

        def calc_ref_attitude_sp(t, T_tilt, h_tilt, J): 
            cos_t = cos(2 * pi * t / T_tilt)
            sin_t = sin(2 * pi * t / T_tilt)
            h_cos_2 = h_tilt * (cos_t + 1) / 2
            h_sin_2 = h_tilt * (sin_t + 1) / 2

            def q(t):
                return np.array([
                    cos(h_cos_2), 
                    sin(h_cos_2), 
                    0, 
                    0]).T
            
            def R(t):
                p0 = 1 - 2 * sin(h_cos_2)**2
                p1 = 2 * cos(h_cos_2) * sin(h_cos_2)
                return np.array([
                    [1, 0, 0],
                    [0, p0, -p1], 
                    [0, p1, p0]
                ])
            
            def W(t):
                p1 = sin(h_cos_2) ** 2
                p2 = cos(h_cos_2) ** 2
                p3 = sin_t
                p0 = ((h_tilt * pi * p3 * p2 * 2) / T_tilt - (h_tilt * pi * p3 * p1 *2) / T_tilt) \
                    * (2 * p1 - 1) \
                    - (8 * h_tilt * pi * p3 *p2 * p1) / T_tilt
                return np.array([
                    p0, 0, 0
                ]).T

            def dW(t):
                p3 = cos_t
                p2 = h_cos_2
                p1 = (h_tilt**2) * pi * (sin(p3)**2) * cos(p2) * sin(p2) * 2 / (T_tilt**2)

                def l1_over_Tilt2(l1p):
                    return h_tilt * pi * cos(p3) * (l1p**2) * 2 / (T_tilt**2)
                
                def l2_over_Tilt2(l2p):
                    return 2 * h_tilt * pi * cos(p2) * sin(p2) / (T_tilt**2)
                
                def l3_over_Tilt(l3p):
                    return 2 * h_tilt * pi * sin(p3) * cos(p2) / T_tilt
                
                l1 = (2 * pi * (l1_over_Tilt2(cos(p2)) + p1) \
                    - 2 * pi * (l1_over_Tilt2(sin(p2)) - p1)) \
                    * (2 * (sin(p2)**2) - 1)
                l2 = - 8 *pi * (l2_over_Tilt2(cos(p3) * cos(p2) * sin(p2)) \
                        + l2_over_Tilt2(h_tilt * (sin(p3)**2) * (sin(p2)**2)) \
                        + l2_over_Tilt2(h_tilt * (sin(p3)**3) * (cos(p2)**2)))
                l3 = - 4 * h_tilt * pi * sin(p3) * cos(p2) * sin(p2) \
                    * (l3_over_Tilt(cos(p2)) - l3_over_Tilt(sin(p2))) / T_tilt
                
                return np.array([
                    l1 + l2 + l3, 0, 0
                ]).T

            Rs = R(t)
            euler = tf.transformations.euler_from_matrix(Rs, 'syxz')
            # Ws = W(t)
            # dWs = dW(t)
            # torque = np.cross(Ws, J @ Ws) + J @ dWs
            Ws = None 
            torque = None
            return euler, Ws, torque
        
        up_time = 2
        hover_time = 10
        land_time = 2
        shutdown_time = 2

        # Ascending param
        h = 1.5
        tau = up_time

        # Tilting param
        T_tilt = 6
        h_tilt = pi / 8
        # h_tilt = pi / 16

        t1 = up_time
        t2 = t1 + hover_time
        t3 = t2 + land_time
        t4 = t3 + shutdown_time

        # positions in NWU
        if t < t1:  # Ascending
            self.pose_target_sp.position.z = takeoff_pose[2] + (
                0.5 * h * (1 - cos(pi * (t - 0) / tau)))
            self.pose_target_sp.velocity.z = 0.5 * h* pi / tau * sin(pi * (t - 0) / tau)
            self.pose_target_sp.acceleration_or_force.z = 0.5 * h * (pi**2) / (tau**2) * cos(pi * (t - 0) / tau)
        elif t > t1 and t < t2: # Hover
            self.pose_target_sp.position.z = takeoff_pose[2] + h

            t_m = (t1 + t2) / 2
            if t > t_m - T_tilt / 2 and t < t_m + T_tilt / 2:
                # Scale for manual control: 1000
                # manual_sp.y = 1000 * h_tilt * (1 + cos(2 * pi / T_tilt * (t - t_m)))
                J = np.eye(3)
                euler, Ws, torque = calc_ref_attitude_sp(t - t_m, T_tilt, h_tilt, J)
                self.manual_sp.x = 1000 * euler[0]
                self.manual_sp.y = 1000 * euler[1]
                self.manual_sp.r = 1000 * euler[2]

                # self.attitude_target_sp.body_rate.x = Ws[0]
                # self.attitude_target_sp.body_rate.y = Ws[1]
                # self.attitude_target_sp.body_rate.z = Ws[2]
                # torque
        elif t > t2 and t < t3: # Descending
            self.pose_target_sp.position.z = takeoff_pose[2] + h - (
                0.5 * h * (1 - cos(pi * (t - t2) / tau)))
            self.pose_target_sp.velocity.z = -0.5 * h* pi / tau * sin(pi * (t - t2) / tau)
            self.pose_target_sp.acceleration_or_force.z = -0.5 * h * (pi**2) / (tau**2) * cos(pi * (t - t2) / tau)
        elif t > t3 and t < t4: 
            self.pose_target_sp.position.z = takeoff_pose[2] - (t - t3)
        elif t > t4:
            self.pose_target_sp.position.z = -10
            should_run = False

        return should_run
    
    """
    Using setpoint_raw to send position target
    """
    def task_8shape(self, t: float, takeoff_pose, should_run):
        
        # Initialization / default values
        self.pose_target_sp.position.x = takeoff_pose[0]
        self.pose_target_sp.position.y = takeoff_pose[1]
        self.pose_target_sp.position.z = takeoff_pose[2]

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
        h = 2.0
        tau = up_time

        # Tilting param
        T_tilt = 2 * pi
        h_tilt = pi / 16
        l = 1

        t1 = up_time
        t2 = t1 + hover_time
        t3 = t2 + land_time
        t4 = t3 + shutdown_time

        # positions in NWU
        if t < t1:  # Ascending
            self.pose_target_sp.position.z = takeoff_pose[2] + (
                0.5 * h * (1 - cos(pi * (t - 0) / tau)))
            self.pose_target_sp.velocity.z = 0.5 * h* pi / tau * sin(pi * (t - 0) / tau)
            self.pose_target_sp.acceleration_or_force.z = 0.5 * h * (pi**2) / (tau**2) * cos(pi * (t - 0) / tau)
        elif t > t1 and t < t2: # Hover
            self.pose_target_sp.position.z = takeoff_pose[2] + h

            t_m = (t1 + t2) / 2
            if t > t_m - T_tilt / 2 and t < t_m + T_tilt / 2:
                idx = 2 * pi / T_tilt * (t - t_m)
                self.pose_target_sp.position.x = l * cos(idx)
                self.pose_target_sp.position.y = l * sin(2 * idx)
                self.pose_target_sp.velocity.x = -1 * pi * l / T_tilt * sin(idx)
                self.pose_target_sp.velocity.y =  2 * pi * l / T_tilt * cos(2 * idx)
                self.pose_target_sp.acceleration_or_force.x = h * (pi**2) / (T_tilt**2) * cos(idx)
                self.pose_target_sp.acceleration_or_force.y = 4 * h * (pi**2) / (T_tilt**2) * sin(idx)

        elif t > t2 and t < t3: # Descending
            self.pose_target_sp.position.z = takeoff_pose[2] + h - (
                0.5 * h * (1 - cos(pi * (t - t2) / tau)))
            self.pose_target_sp.velocity.z = -0.5 * h* pi / tau * sin(pi * (t - t2) / tau)
            self.pose_target_sp.acceleration_or_force.z = -0.5 * h * (pi**2) / (tau**2) * cos(pi * (t - t2) / tau)
        elif t > t3 and t < t4: 
            self.pose_target_sp.position.z = takeoff_pose[2] - (t - t3)
        elif t > t4:
            self.pose_target_sp.position.z = -10
            should_run = False

        return should_run
