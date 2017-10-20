#!/usr/bin/env python

import rospy
import actionlib

from TBLogic import TBLogic
from std_msgs.msg import Empty
from tb_base.srv import PoiDetect, MapExplored
from geometry_msgs.msg import Point

import sys



if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("usage: x1 y1 x2 y2")
        p1 = Point(8,2,0)
        p2 = Point(11,3,0)
    else:
        p1 = Point(float(sys.argv[1]),float(sys.argv[2]),0.0)
        p2 = Point(float(sys.argv[3]),float(sys.argv[4]),0.0)
        rospy.init_node('DrivePathNode')
        xx = TBLogic()
        c = xx.get_map_pos()
        print c
        path = xx.get_path(c.position,p1)
        #while not rospy.is_shutdown():
        #zuerst vom ausgangspunkt zum start punkt fahren
        while not (c.position.x == p1.x and c.position.y == p1.y):
            if rospy.is_shutdown():
                break
            path = xx.get_path(c.position,p1)
            xx.turn_to_next_point(path.path[1])
            xx.drive_forward(38)
            c = xx.get_map_pos()
        #zuerst vom ausgangspunkt zum start punkt fahren
        while not (c.position.x == p2.x and c.position.y == p2.y):
            if rospy.is_shutdown():
                break
            path = xx.get_path(c.position,p2)
            xx.turn_to_next_point(path.path[1])
            xx.drive_forward(38)
            c = xx.get_map_pos()
        if rospy.is_shutdown():
            print "Abgebrochen ROS Shutdown"
        else:
            print "Ziel erreicht"   