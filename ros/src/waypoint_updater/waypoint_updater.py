#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number default:200


class WaypointUpdater(object):
    def __init__(self):
        # init node
        rospy.init_node('waypoint_updater')
        
        # subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint',?,self.obstacle_cb)
    
        # publisher final waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.base_waypoints = Lane()
        self.current_pose = PoseStamped()
        self.current_twist = TwistStamped()
        self.base_waypoints_cb = False
        self.current_pose_cb = False
        self.current_twist_cb = False
        self.wp_num = 0
        self.wp_front = 0
        self.red_light_index = -1        # store the waypoint index of the upcoming red lights position
        
        # publishing loop
        self.pub_waypoints()
        
        # spin node
        rospy.spin()

    def pub_waypoints(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            # rospy.loginfo("publishing waypoints .... ")
            # check if we recieved the waypoint and current vehicle data
            if((len(self.base_waypoints.waypoints) > 0) & self.base_waypoints_cb & self.current_pose_cb):
                # find the first waypoint in front of the current vehicle position
                front_index = self.nearest_front()
                # self.next_front() has a bug on the back part of the track, where the next way-points just iterate
                # independent of the cars position.
                #front_index = self.next_front()

                desired_vel = self.set_linear_velocity(front_index)

                rospy.loginfo("current waypoint index .... %d", front_index)
                rospy.loginfo("current waypoint x .... %f", self.base_waypoints.waypoints[front_index].pose.pose.position.x)
                rospy.loginfo("current waypoint y .... %f", self.base_waypoints.waypoints[front_index].pose.pose.position.y)
                rospy.loginfo("current linear twist .... %f", self.base_waypoints.waypoints[front_index].twist.twist.linear.x)
                rospy.loginfo("current x .... %f", self.current_pose.pose.position.x)
                rospy.loginfo("current y .... %f", self.current_pose.pose.position.y)
                rospy.loginfo("red light waypoint index %d", self.red_light_index)
                
                # copy look ahead waypoints
                self.base_waypoints.waypoints[front_index].twist.twist.linear.x = desired_vel
                final_waypoints = Lane()
                final_waypoints.header = self.base_waypoints.header
                for i in range(front_index, front_index+LOOKAHEAD_WPS):
                    ci = i%self.wp_num
                    final_waypoints.waypoints.append(self.base_waypoints.waypoints[ci])
 
                self.final_waypoints_pub.publish(final_waypoints)
         
            rate.sleep()
            
    @staticmethod      
    def Euclidean(x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
    ## Finding the next waypoint 1: Vector projection         
    def nearest_front(self):
        # this function finds where we are in from the closest distance
        # some start up variables
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        wp_x = self.base_waypoints.waypoints[0].pose.pose.position.x
        wp_y = self.base_waypoints.waypoints[0].pose.pose.position.y
        min_d = self.Euclidean(wp_x,wp_y,current_x,current_y)
        min_i = 0
        lookup_start = 1
        lookup_end = len(self.base_waypoints.waypoints)
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
        wp_1 = (min_i-1)%self.wp_num
        wp_2 =  min_i%self.wp_num
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
        seg_length = math.sqrt(dx*dx + dy*dy)
        u = (Rx*dx + Ry*dy)/seg_length
        
        # check index from vector projection if already pass this point
        # also check cyclic index
        if abs(u) > seg_length:
            wp_front+=1
            
        # record this for the next cycle
        self.wp_front = wp_front%self.wp_num     
        return wp_front%self.wp_num
    
    # Find the next waypoint 2: Simple closest point
    def next_front(self):
        # this function finds where we are in from the closest distance
        # some start up variables
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
        """ calculates the desired speed of the vehicle and ramps down the velocity
        when stopping at red lights.

        Args:
            index (int): the index of the closest point of the stop lights

        """

        desired_vel = self.base_waypoints.waypoints[index].twist.twist.linear.x
        dist_x = dist_y = dist = 0

        wheel_base = rospy.get_param('~wheel_base', 2.8498)

        if(self.red_light_index >= 0):
            dist_x = self.base_waypoints.waypoints[self.red_light_index-1].pose.pose.position.x - \
                     self.base_waypoints.waypoints[index].pose.pose.position.x
            dist_y = self.base_waypoints.waypoints[self.red_light_index-1].pose.pose.position.y - \
                     self.base_waypoints.waypoints[index].pose.pose.position.y

            # calculate the distance to target location from the front of the vehicle
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2) - wheel_base

            if (dist < 0.2):
                desired_vel = 0.
            elif(dist < 45): # ramp the velocity down when close to the stop point
                desired_vel = 11.111 * dist / 45.0 # simple ramp function
        else:
            desired_vel = 11.111

        return desired_vel
    
    
    def pose_cb(self, msg):
        # Simulator must connected! for receiving the message
        if not self.current_pose_cb:
            self.current_pose_cb = True
        self.current_pose = msg


    def twist_cb(self, msg):
        # Simulator must connected! for receiving the message
        if not self.current_twist_cb:
            self.current_twist_cb = True
        self.current_twist = msg


    def waypoints_cb(self, msg):
        # rospy.loginfo("waypoint msg receive with %d length.... ", len(msg.waypoints))
        if not self.base_waypoints_cb:
            self.base_waypoints_cb = True   
        self.base_waypoints = msg
        self.wp_num = len(msg.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
