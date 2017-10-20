#!/usr/bin/env python

import rospy
import actionlib

from TBLogic import TBLogic
from std_msgs.msg import Empty
from tb_base.srv import PoiDetect, MapExplored


if __name__ == '__main__':
    rospy.init_node('TBLogicNode')
    xx = TBLogic()
    # TODO: remove when ready, testing only
    while not rospy.is_shutdown():
        p = xx.check_laser()
        if p.front:
            print xx.check_poi()
        if not p.right:
            print "Drehe rechts"
            xx.turn_right()
            xx.drive_forward(38)
        elif not p.front:
            print "Fahre vorwaerts"
            xx.drive_forward(38)
        else:
            print "Drehe links"
            xx.turn_left()
        if xx.check_explored():
            print "Labyrinth vollstaendig erkundet !"
            break
    rospy.spin()