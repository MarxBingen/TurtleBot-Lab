#!/usr/bin/env python

import rospy
import actionlib


from tb_base.msg import DriveForwardAction, DriveForwardActionGoal, TurnAroundActionGoal, TurnAroundAction
from tb_base.msg import WallDetection


class TBLogic(object):

    drive_forward_client = None

    def __init__(self):
        self.drive_forward_client = actionlib.SimpleActionClient(
            'DriveForward', DriveForwardAction)
        self.turn_around_client = actionlib.SimpleActionClient(
            'TurnAround', TurnAroundAction)
        self.drive_forward_client.wait_for_server()
        self.turn_around_client.wait_for_server()

    def drive_forward(self, strecke):
        goal = DriveForwardActionGoal()
        goal.goal.distance = strecke
        goal.goal.speed = 0.2
        # Sends the goal to the action server.
        self.drive_forward_client.send_goal(goal.goal)
        # Waits for the server to finish performing the action.
        completed = self.drive_forward_client.wait_for_result()
        if completed:
            print self.drive_forward_client.get_result()
        return completed

    def turn_around(self, winkel):
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
