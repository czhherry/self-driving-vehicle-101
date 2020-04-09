#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path

import math
import numpy as np
from scipy.spatial import KDTree


LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/smart/rear_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_path_pub = rospy.Publisher('final_path', Path, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.pose = None

        # rospy.spin()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        return closest_idx          

    def publish_waypoints(self, closest_idx):
    	# fill in final waypoints to publish
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

        # fill in path for visulization in Rviz
        path = Path()
        path.header.frame_id = '/world'
        for p in lane.waypoints:
	        path_element = PoseStamped()
	        path_element.pose.position.x = p.pose.pose.position.x
	        path_element.pose.position.y = p.pose.pose.position.y
	        path.poses.append(path_element)

        self.final_waypoints_pub.publish(lane)
        self.final_path_pub.publish(path)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
