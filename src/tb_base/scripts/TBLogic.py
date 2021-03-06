#!/usr/bin/env python

import rospy
import actionlib


from tb_base.msg import DriveForwardAction, DriveForwardActionGoal, TurnAroundActionGoal, TurnAroundAction
from tb_base.msg import WallDetection
from std_msgs.msg import Empty
from tb_base.srv import PoiDetect, MapExplored, MapPos, PathFind, PathFindRequest


class TBLogic(object):

    drive_forward_client = None
    turn_around_client = None
    poi_service = None

    def __init__(self):
        #Odom resetten
        odom_reset = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty , queue_size=1)
        odom_reset.publish(Empty())
        odom_reset.unregister()
        self.drive_forward_client = actionlib.SimpleActionClient(
            'DriveForward', DriveForwardAction)
        self.turn_around_client = actionlib.SimpleActionClient(
            'TurnAround', TurnAroundAction)
        self.drive_forward_client.wait_for_server()
        self.turn_around_client.wait_for_server()

        rospy.wait_for_service('PoiDetectService')
        self.poi_service = rospy.ServiceProxy('PoiDetectService', PoiDetect)

        rospy.wait_for_service('MapServiceExplored')
        self.map_explored_service = rospy.ServiceProxy('MapServiceExplored', MapExplored)

        rospy.wait_for_service('MapServicePos')
        self.map_pos_service = rospy.ServiceProxy('MapServicePos', MapPos)

        rospy.wait_for_service('PathFind')
        self.path_service = rospy.ServiceProxy('PathFind', PathFind)

    def drive_forward(self, strecke):
        '''faehrt vorwaerst, eigene implementierung faehrt weiter, auch wenn strecke
        gefahren, muss also zum stop gecancelled werden
        AKTUELL deaktiviert'''
        rospy.sleep(2)
        goal = DriveForwardActionGoal()
        goal.goal.distance = strecke
        goal.goal.speed = 0.2
        completed = self.drive_forward_client.send_goal_and_wait(goal.goal)
        if completed:
            print self.drive_forward_client.get_result()
        return completed

    def __turn_around(self, winkel):
        #es wird eine etwaige forward-action gestoppt
        self.stop_driving()
        goal = TurnAroundActionGoal()
        goal.goal.degrees = winkel
        goal.goal.speed = 45
        completed = self.turn_around_client.send_goal_and_wait(goal.goal)
        if completed:
            print self.turn_around_client.get_result()
        return completed

    def check_laser(self):
        feldbelegung = rospy.wait_for_message(
            'wallDetection', WallDetection, 2.0)
        return feldbelegung

    def stop_driving(self):
        self.drive_forward_client.cancel_all_goals()

    def turn_left(self):
        self.__turn_around(90)

    def turn_right(self):
        self.__turn_around(-90)

    def check_poi(self):
        result = self.poi_service()
        return result

    def check_explored(self):
        result = self.map_explored_service()
        print result
        return result.explored

    def get_map_pos(self):
        result = self.map_pos_service()
        return result.position

    def get_path(self, start, ziel):
        req = PathFindRequest()
        req.start = start
        req.ziel = ziel
        result = self.path_service(req)
        return result

    def turn_to_next_point(self, p):
        c = self.get_map_pos()
        #westen oder osten
        if (c.position.x == p.x):
            #nach westen
            if (p.y > c.position.y):
                if c.richtung == 'NORD':
                    self.__turn_around(90)
                elif c.richtung == 'OST':
                    self.__turn_around(180)
                elif c.richtung == 'SUED':
                    self.__turn_around(-90)
            #nach osten
            else:
                if c.richtung == 'NORD':
                    self.__turn_around(-90)
                elif c.richtung == 'WEST':
                    self.__turn_around(180)
                elif c.richtung == 'SUED':
                    self.__turn_around(90)
        #norden oder sueden
        else:
            #nach Norden
            if (p.x > c.position.x):
                if c.richtung == 'WEST':
                    self.turn_right()
                elif c.richtung == 'OST':
                    self.turn_left()
                elif c.richtung == 'SUED':
                    self.__turn_around(180)
            else:
                if c.richtung == 'WEST':
                    self.turn_left()
                elif c.richtung == 'OST':
                    self.turn_right()
                elif c.richtung == 'NORD':
                    self.__turn_around(180)


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
