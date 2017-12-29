#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, Light, TrafficLight
from std_msgs.msg import Bool
import math
from copy import deepcopy

##
from std_msgs.msg import Int32              # traffic waypoint
##

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status (my note: RED/AMBER/GREEN) in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''
#reduced the Number of waypoints from 200 to 20
LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number default:200


class WaypointUpdater(object):
    def __init__(self):
        # init node
        rospy.init_node('waypoint_updater')
        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Light, self.traffic_cb)
        
        # subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        #rospy.Subscriber('/obstacle_waypoint',?,self.obstacle_cb)
    
        # publisher final waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.base_waypoints = Lane() # data from the map used to set the environments desired position
        self.map_waypoints = Lane() # data used to control the cars desired position
        self.current_pose = PoseStamped()
        self.current_twist = TwistStamped()
        self.base_waypoints_cb = False
        self.current_pose_cb = False
        self.current_twist_cb = False
        self.wp_num = 1
        self.wp_front = None
        self.light_index = -1   # store the waypoint index of the upcoming lights position
        self.light_state = TrafficLight.UNKNOWN
        self.stop_at_light = False

        self.desired_vel = 0.0 # the desired vehicle velocity at each timestep
        #self.max_vel = 10.0 # m/s
        self.ramp_dist = 30 # distance to ramp up and down the acceleration (m)
        # Kinematics => Vf^2 = Vi^2 + 2*a*d => Vi = 0
        self.acceleration_rate = 0.75/30.  #self.max_vel / (2 * self.ramp_dist)

        self.dbw = False  # dbw enable
        self.dbw_init = False  # first connection established(in syn with publish loop)

        # spin node
        rospy.spin()

    def pub_waypoints(self):
        """
        Find the next waypoint parameters and publises them to self.final_waypoints_pub

        """

        # check if we recieved the waypoint and current vehicle data
        if(self.dbw & (len(self.base_waypoints.waypoints) > 0) & self.base_waypoints_cb & self.current_pose_cb):
            # find the first waypoint in front of the current vehicle position
            front_index = self.nearest_front()
            # self.next_front() has a bug on the back part of the track, where the next way-points just iterate
            # independent of the cars position.
            #front_index = self.next_front()

            self.set_linear_velocity(max(front_index, 0))

            rospy.loginfo("current waypoint index .... %d", front_index)
            rospy.loginfo("current waypoint x: %f ... y: %f",
                          self.base_waypoints.waypoints[front_index].pose.pose.position.x,
                          self.base_waypoints.waypoints[front_index].pose.pose.position.y)
            rospy.loginfo("current linear velocity: %f ... map: %f ... desired: %f",
                          self.current_twist.twist.linear.x,
                          self.map_waypoints.waypoints[front_index].twist.twist.linear.x,
                          self.desired_vel)
            rospy.loginfo("current pose x: %f ... y: %f", self.current_pose.pose.position.x,
                          self.current_pose.pose.position.y)
            rospy.loginfo("light state: %d ... light waypoint index %d", self.light_state, self.light_index)

            # copy look ahead waypoints
            final_waypoints = Lane()
            final_waypoints.header = self.base_waypoints.header

            for i in range(front_index, front_index+LOOKAHEAD_WPS):
                ci = i%self.wp_num
                final_waypoints.waypoints.append(self.base_waypoints.waypoints[ci])
                final_waypoints.waypoints[-1].twist.twist.linear.x = self.desired_vel

            self.final_waypoints_pub.publish(final_waypoints)
        else:
            # Reset the current waypoint so when autonomous mode is enabled the
            # vehicle finds the nearest waypoint
            self.wp_front = None

            
    @staticmethod      
    def Euclidean(x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
    ## Finding the next waypoint 1: Vector projection         
    def nearest_front(self):
        """
        Finds where the nearest map waypoint is based off the vehicle
        current known position.

        """

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        wp_x = self.base_waypoints.waypoints[0].pose.pose.position.x
        wp_y = self.base_waypoints.waypoints[0].pose.pose.position.y
        min_d = self.Euclidean(wp_x,wp_y,current_x,current_y)
        min_i = 0
        
        # On start up we search through the list
        if self.wp_front is None:  
            lookup_start = 0
            lookup_end = len(self.base_waypoints.waypoints)
        # Continue from the current location + look ahead points
        elif self.wp_front + LOOKAHEAD_WPS > len(self.base_waypoints.waypoints):
            if self.wp_front >= len(self.base_waypoints.waypoints):
                # cycles the waypoints when back at the start location
                lookup_start = 0
                lookup_end = lookup_start + LOOKAHEAD_WPS
            else:
                lookup_start = self.wp_front
                lookup_end = len(self.base_waypoints.waypoints)
        else:
            lookup_start = self.wp_front
            lookup_end = lookup_start+LOOKAHEAD_WPS
        
        for i in range(lookup_start,lookup_end):
            wp_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.base_waypoints.waypoints[i].pose.pose.position.y
            dist_i = self.Euclidean(wp_x,wp_y,current_x,current_y)
            if (dist_i < min_d):
                min_d = dist_i
                min_i = i
        
        # find the front way point in front of the car using vector projection
        # between the closest wap point[i] and [i-1]
        # Note: cyclic waypoint
        wp_front = min_i

        if(self.wp_num != 0):
            wp_1 = (min_i-1)%self.wp_num
            wp_2 = min_i % self.wp_num
        else:
            wp_1 = min_i - 1
            wp_2 = min_i

        x1 = self.base_waypoints.waypoints[wp_1].pose.pose.position.x
        y1 = self.base_waypoints.waypoints[wp_1].pose.pose.position.y
        x2 = self.base_waypoints.waypoints[wp_2].pose.pose.position.x
        y2 = self.base_waypoints.waypoints[wp_2].pose.pose.position.y
        x  = self.current_pose.pose.position.x
        y  = self.current_pose.pose.position.y
        Rx = x-x1
        Ry = y-y1
        dx = x2-x1
        dy = y2-y1
        seg_length = max(math.sqrt(dx*dx + dy*dy), 1.) # prevent divide by 0
        u = (Rx*dx + Ry*dy)/seg_length
        
        # check index from vector projection if already pass this point
        # also check cyclic index
        if abs(u) > seg_length:
            wp_front+=1
            
        # record this for the next cycle
        self.wp_front = wp_front%self.wp_num     
        return wp_front%self.wp_num
    
    # (Deprecated) Find the next waypoint 2: Simple closest point
    def next_front(self):
        """
        Finds where the next map waypoint based off the vehicles previously
        known position.

        """

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        wp_x = self.base_waypoints.waypoints[0].pose.pose.position.x
        wp_y = self.base_waypoints.waypoints[0].pose.pose.position.y
        min_d = self.Euclidean(wp_x,wp_y,current_x,current_y)
        min_i = 0
        lookup_start = self.wp_front
        lookup_end = lookup_start + LOOKAHEAD_WPS #len(self.base_waypoints.waypoints)
        for i in range(lookup_start,lookup_end):
            wp_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.base_waypoints.waypoints[i].pose.pose.position.y
            dist_i = self.Euclidean(wp_x,wp_y,current_x,current_y)
            if (dist_i < min_d):
                min_d = dist_i
                min_i = i
        
        # select the point in front (negative velocity if the point goes backward!)
        # Note: cyclic waypoint
        wp_front =  min_i%self.wp_num
        x1 = self.base_waypoints.waypoints[wp_front].pose.pose.position.x
        # if already pass this way point considering the x direction -> use the next one
        if(current_x >= x1):
            wp_front += 1
                  
        # record this for the next cycle
        self.wp_front = wp_front%self.wp_num     
        return wp_front%self.wp_num


    def set_linear_velocity(self, index):
        """
        Calculates the desired speed of the vehicle and ramps down the velocity
        when stopping at red lights.

        Args:
            index (int): the index of the closest point of the stop lights

        """

        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        # set the environments speed limit and slow down by 10%
        base_vel = self.map_waypoints.waypoints[index].twist.twist.linear.x * 0.9

        state = self.light_state
        min_vel = 1.
        overshoot_dist = -0.5

        if(state == TrafficLight.RED or state == TrafficLight.YELLOW): #self.light_index >= 0):
            dist_x = self.base_waypoints.waypoints[self.light_index].pose.pose.position.x - \
                     self.base_waypoints.waypoints[index].pose.pose.position.x
            dist_y = self.base_waypoints.waypoints[self.light_index].pose.pose.position.y - \
                     self.base_waypoints.waypoints[index].pose.pose.position.y

            current_vel = self.current_twist.twist.linear.x

            # calculate the distance to target location from the front of the vehicle
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2) - wheel_base

            # brake at double the acceleration rate
            speed_check = current_vel #/ (self.acceleration_rate*30)
            dist_check = abs(dist + 2)
            speed_stop_check = (speed_check < dist_check) or (current_vel < 3. and dist > overshoot_dist)
            rospy.loginfo("Can stop at light: %f   <   dist: %f ... %r", speed_check, dist_check, speed_stop_check)

            if(dist > overshoot_dist and abs(dist) < self.ramp_dist and speed_stop_check):
                # ramp the velocity down when close to the stop point

                if (dist < 1.):
                    self.desired_vel = 0.
                else:
                    self.desired_vel = max(base_vel * dist / self.ramp_dist, min_vel) # simple ramp function

                self.stop_at_light = True

            else:
                self.desired_vel = base_vel
                self.stop_at_light = False

            rospy.loginfo("state: %d ... base_vel: %f ... dist: %f ... desired_vel: %f ... stop at light: %r",
                          state, base_vel, dist, self.desired_vel, self.stop_at_light)
        else:
            # ramp speed up with acceleration of 0.041(m/s)/ros_rate
            self.desired_vel = max(self.desired_vel + self.acceleration_rate, min_vel)
            self.stop_at_light = False

        self.desired_vel = min(self.desired_vel, base_vel)

    
    def pose_cb(self, msg):
        # Simulator must connected! for receiving the message
        if not self.current_pose_cb:
            self.current_pose_cb = True
        self.current_pose = msg
        self.pub_waypoints()

    def twist_cb(self, msg):
        # Simulator must connected! for receiving the message
        if not self.current_twist_cb:
            self.current_twist_cb = True
        self.current_twist = msg


    def waypoints_cb(self, msg):
        rospy.loginfo("waypoint msg receive with %d length.... ", len(msg.waypoints))
        if not self.base_waypoints_cb:
            self.base_waypoints_cb = True   
        self.base_waypoints = msg
        self.map_waypoints = deepcopy(msg)
        self.wp_num = len(msg.waypoints)

    def traffic_cb(self, msg):
        self.light_index = msg.waypoint.data
        self.light_state = msg.state.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def dbw_enabled_cb(self, msg):
        if not self.dbw_init:
            self.dbw_init = True
        self.dbw = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
