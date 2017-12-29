import rospy
from pid import PID
from lowpass import *
from yaw_controller import YawController
from math import *

##
#    Throttle PID
##
Kp_v = 1.25      #0.03      # 0.08      # 0.05
Kd_v = 1.     #0.01      # 0.02      # 0.02
Ki_v = 0.1    #.002     # 0.005     # 0.001

##
#    Steering PID
##
Kp_s = 1.0
Kd_s = 0.2
Ki_s = 0.001


##
#    Vehicle throttle control setting
## 
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_THROTTLE = -2.0
MAX_THROTTLE = 1.0

MIN_INTEGRAL = -10.  # this will max the integral error to only contribute 0.2 to the throttle
MAX_INTEGRAL = 10.
# mynote: implement a feedback controller from pid,low pass (both for acceleration), yaw controller (steering)

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_cfg = args[0]
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.dbw_status = False
        self.throttle = PID(Kp_v, Ki_v, Kd_v, mn=MIN_THROTTLE, mx=MAX_THROTTLE, min_i=MIN_INTEGRAL, max_i=MAX_INTEGRAL)
        self.error_v  = 0.0
        self.wheel_base    = self.vehicle_cfg['wheel_base']
        self.steer_ratio   = self.vehicle_cfg['steer_ratio']
        self.min_speed     = 0.0
        self.max_lat_accel = self.vehicle_cfg['max_lat_accel']
        self.max_steer_angle = self.vehicle_cfg['max_steer_angle']
        self.vehicle_mass  = self.vehicle_cfg['vehicle_mass']
        self.wheel_radius  = self.vehicle_cfg['wheel_radius']

        self.steering = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steer_pid = PID(Kp_s, Ki_s, Kd_s, mn=-self.max_steer_angle, mx=self.max_steer_angle,
                             min_i=-self.max_steer_angle, max_i=self.max_steer_angle)

        self.vel_filter = LowPassFilter(29, 1)  # use only 14.29% of latest error
        self.steer_filter = LowPassFilter(14, 1)  # use only 10%

        self.highest_v = 0


    def control(self, *args, **kwargs):
        # Change the arg, kwarg list to suit your needs (args: target_v,target_w,current_v,dbw_status,dt)
        # Feedback controls: pid/lowpass (throttle) and yaw controller (steering)
        # Throttle := [0,1], Brake := N*m, Steering := Radian
        
        # velocity args
        target_v   = args[0]#target_velocity.twist.linear
        target_w   = args[1]#target_rotation.twist.angular
        current_v  = args[2]#current_velocity.twist.linear
        current_w  = args[3]#current_rotation.twist.angular
        dbw_status = args[4]#dbw
        dt         = args[5]#sample time dt
        # linear speed error in x
        v_current = self.vel_filter.filt(abs(current_v.x))
        v_target  = abs(target_v.x)
        v_error   = v_target - v_current

        self.highest_v = max(self.highest_v, current_v.x)
        rospy.loginfo("highest velocity .... %f ", self.highest_v)

        throttle_cmd = 0.    # Throttle command value
        brake_cmd = 0.     # Throttle command value

        d_t = 2.0  # desire braking time: reduce to the target speed within d_t
        # over the limit, then we need to apply brake

        # Throttle and brake control
        if(current_v.x < 2.) and (target_v.x == 0.):
            throttle_cmd = 0.
            self.throttle.reset()
            brake_cmd = 1000.  #Nm
        else:
            throttle_cmd = self.throttle.step(v_error, dt)

            # allow a small margin of error before applying the brakes
            if (throttle_cmd < 0.):  #v_error < -0.25):
                gain = abs(throttle_cmd) * current_v.x / 10.  #min(max(abs(v_error * throttle_cmd) * current_v.x/5, 0.1), 4)
                # d_speed is negative so need to reverse it to positive
                Bf = self.vehicle_mass * 1. * 9.81
                brake_cmd = fabs(Bf * self.wheel_radius * gain)  #self.vehicle_mass * self.wheel_radius * gain / d_t)
                rospy.loginfo("throttle_cmd: %f ... velocity error: %f ... delta t: %f ... brake gain: %f",
                              throttle_cmd, v_error, dt, gain)

            throttle_cmd = max(throttle_cmd, 0.0)

        # Steering angle- steering output proportional to (v_current/v_target)
        #               - something fishy about this yaw control scheme
        #               - what happen if v_current > v_target much a lot? The steering might spin out of control!
        #               - so we need to make sure the speed doesn't go pass the limit for too long & lowpass filter also help
        w_target  = target_w.z
        w_current = 0.  #self.steer_filter.filt(current_w.z)
        steering_error = w_target #- w_current
        steering_cmd = self.steering.get_steering(v_target, steering_error, v_current)
        steering_cmd = self.steer_pid.step(steering_cmd, dt)

        rospy.loginfo("velocity target: %f ... current: %f ", v_target, v_current)
        rospy.loginfo("steer target: %f ... current: %f ", w_target, w_current)
        rospy.loginfo("throttle: %f ... brake: %f ... Steer: %f", throttle_cmd, brake_cmd, steering_cmd)
        
        # Return throttle, brake, steer in this order
        return throttle_cmd, brake_cmd, steering_cmd
