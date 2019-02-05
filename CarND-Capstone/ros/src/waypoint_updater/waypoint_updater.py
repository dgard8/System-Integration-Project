#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        self.decel_limit = -rospy.get_param('~decel_limit', -5)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO later: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.waypoints2D = None
        self.waypointsTree = None
        self.pose = None
        self.red_light_waypoint = -1

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoints:
                closestWaypointIndex = self.getClosestWaypoint()
                self.publishWaypoints(closestWaypointIndex)
            rate.sleep()
            
    def getClosestWaypoint(self):
        print(self.pose)
        x = self.pose.position.x
        y = self.pose.position.y
        closestIndex = self.waypointsTree.query([x,y], 1)[1]
        
        # is this behind or ahead of us? Do some math to find out
        closestCoordinate = self.waypoints2D[closestIndex]
        previousCoordinate = self.waypoints2D[closestIndex-1]
        
        # turn these into vectors
        closestVector = np.array(closestCoordinate)
        previousVector = np.array(previousCoordinate)
        currentVector = np.array([x,y])
        
        # take the dot product. If it is poitive the two vectors are in the same direction
        # and the car is ahead of the closest point. Otherwise it is behind.
        dotProduct = np.dot(closestVector - previousVector, currentVector - closestVector)
        
        if dotProduct>0:
            closestIndex = (closestIndex + 1) % len(self.waypoints2D)
            
        return closestIndex
    
    def publishWaypoints(self, closestWaypointIndex):
        lane = Lane()
        original_waypoints = self.waypoints[closestWaypointIndex:closestWaypointIndex+LOOKAHEAD_WPS]
        rospy.loginfo("current waypoint: %s. red light waypoint: %s", closestWaypointIndex, self.red_light_waypoint)
        distanceToRedLight = self.red_light_waypoint - closestWaypointIndex
        if 0 < distanceToRedLight < LOOKAHEAD_WPS:
            rospy.loginfo("distance to red light: %s. current velocity: %s",distanceToRedLight, self.get_waypoint_velocity(original_waypoints[0]))
            new_waypoints = []
            for i in range(distanceToRedLight):
                new_waypoint = Waypoint()
                new_waypoint.pose = original_waypoints[i].pose
                distance = self.distance(original_waypoints, i, distanceToRedLight - 2)
                new_velocity = math.sqrt(2 * self.decel_limit * distance)
                if new_velocity < 1:
                    new_velocity = 0
                new_velocity = min(new_velocity, self.get_waypoint_velocity(original_waypoints[i]))
                new_waypoint.twist.twist.linear.x = new_velocity
                rospy.loginfo("red light velocity for %s waypoint: %s",i,new_velocity)
                new_waypoints.append(new_waypoint)
            lane.waypoints = new_waypoints
        else:
            lane.waypoints = original_waypoints
                
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        print(msg)
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.waypoints2D = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in msg.waypoints]
        self.waypointsTree = KDTree(self.waypoints2D)

    def traffic_cb(self, msg):
        self.red_light_waypoint = msg.data

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
