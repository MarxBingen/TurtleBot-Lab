#!/usr/bin/env python

import rospy
import actionlib


from tb_base.msg import DriveForwardAction, DriveForwardActionGoal, TurnAroundActionGoal, TurnAroundAction
from tb_base.msg import WallDetection
from std_msgs.msg import Empty


class TBLogic(object):

    drive_forward_client = None

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

    def drive_forward(self, strecke):
        '''faehrt vorwaerst, eigene implementierung faehrt weiter, auch wenn strecke
        gefahren, muss also zum stop gecancelled werden'''
        goal = DriveForwardActionGoal()
        goal.goal.distance = strecke
        goal.goal.speed = 0.2
        goal.goal.stop = False
        # Sends the goal to the action server.
        self.drive_forward_client.send_goal(goal.goal)
        # Waits for the server to finish performing the action.
        completed = self.drive_forward_client.wait_for_result()
        if completed:
            print self.drive_forward_client.get_result()
        return completed

    def turn_around(self, winkel):
        #es wird eine etwaige forward-action gestoppt
        self.stop_driving()
        goal = TurnAroundActionGoal()
        goal.goal.degrees = winkel
        goal.goal.speed = 45
        # Sends the goal to the action server.
        self.turn_around_client.send_goal(goal.goal)
        # Waits for the server to finish performing the action.
        completed = self.turn_around_client.wait_for_result()
        if completed:
            print self.turn_around_client.get_result()
        return completed

    def check_laser(self):
        feldbelegung = rospy.wait_for_message(
            'wallDetection', WallDetection, 2.0)
        return feldbelegung

    def stop_driving(self):
        goal = DriveForwardActionGoal()
        goal.goal.distance = 0
        goal.goal.speed = 0.2
        goal.goal.stop = True
        # Sends the goal to the action server.
        self.drive_forward_client.send_goal(goal.goal)
        # Waits for the server to finish performing the action.
        completed = self.drive_forward_client.wait_for_result()
        if completed:
            print self.drive_forward_client.get_result()
        return completed

    def turn_left(self):
        self.turn_around(90)

    def turn_right(self):
        self.turn_around(-90)


if __name__ == '__main__':
    rospy.init_node('TBLogicNode')
    xx = TBLogic()
    # TODO: remove when ready, testing only
    while not rospy.is_shutdown():
        p = xx.check_laser()
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
    rospy.spin()
