from pid import PID
from lowpass import *
from yaw_controller import YawController
from math import *

##
#    Throttle PID
##
Kp_v = 2.0      #0.03      # 0.08      # 0.05
Kd_v = 2.25     #0.01      # 0.02      # 0.02
Ki_v = 0.045    #.002     # 0.005     # 0.001

##
#    Steering PID
##
Kp_s = 1.5
Kd_s = 0.2
Ki_s = 0.001


##
#    Vehicle throttle control setting
## 
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0

MIN_INTEGRAL = -30.0  # this will max the integral error to only contribute 0.6 to the throttle
MAX_INTEGRAL = 30.0
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

        self.vel_filter = LowPassFilter(6, 1)  # use only 14.29% of latest error
        self.steer_filter = LowPassFilter(1, 3)  # use only 75%


    def control(self, *args, **kwargs):
        # Change the arg, kwarg list to suit your needs (args: target_v,target_w,current_v,dbw_status,dt)
        # Feedback controls: pid/lowpass (throttle) and yaw controller (steering)
        # Throttle := [0,1], Brake := N*m, Steering := Radian
        
        # velocity args
        target_v   = args[0]#current_velocity.twist.linear
        target_w   = args[1]#current_velocity.twist.angular
        current_v  = args[2]#twist_cmd.twist.linear
        dbw_status = args[3]#dbw
        dt         = args[4]#sample time dt
        # linear speed error in x
        v_current = self.vel_filter.filt(abs(current_v.x))
        v_target  = abs(target_v.x)
        v_error   = v_target - v_current

        # Throttle control
        throttle_cmd = self.throttle.step(v_error, dt)
        
        # Steering angle- steering output proportional to (v_current/v_target)
        #               - something fishy about this yaw control scheme
        #               - what happen if v_current > v_target much a lot? The steering might spin out of control!
        #               - so we need to make sure the speed doesn't go pass the limit for too long & lowpass filter also help
        w_target = target_w.z
        steering_error = self.steer_filter.filt(w_target)
        steering_cmd = self.steering.get_steering(v_target, steering_error, v_current)
        steering_cmd = self.steer_pid.step(steering_cmd, dt)
        #steering_cmd = (steering_cmd+pi)%(2*pi) - pi
        print("steering: ", steering_cmd)
        
        print("target v.x: ", target_v.x)
        print("target v.y: ", target_v.y)
        print("target v.z: ", target_v.z)
        print("target w.x: ", target_w.x)
        print("target w.y: ", target_w.y)
        print("target w.z: ", target_w.z)
        
        print("current v.x: ", current_v.x)
        print("current v.y: ", current_v.y)
        print("current v.z: ", current_v.z)
        print("filtered v.x: ", v_current)
        
        # NOTE: Just to get thing going only:need to fine tune this
        #       Or use different approach altogether
        # Brake control                                 
        #d_speed = v_target - v_current
        brake_cmd = 0.0
        d_t = 2.0   # desire braking time: reduce to the target speed within d_t
        # over the limit, then we need to apply brake
        # allow a small margin of error before applying the brakes
        if(v_error < -0.5):
            # d_speed is negative so need to reverse it to positive
            brake_cmd = -self.vehicle_mass*self.wheel_radius*v_error/d_t
        
        
        print("throttle: ", throttle_cmd)
        print("brake: ", brake_cmd)
        print("steering: ", steering_cmd)
        
        # Return throttle, brake, steer in this order
        return throttle_cmd, brake_cmd, steering_cmd
