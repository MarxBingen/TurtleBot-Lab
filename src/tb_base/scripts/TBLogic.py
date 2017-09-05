#!/usr/bin/env python

import rospy
import sys
import time
import math
import numpy as np
import tf
import actionlib


from tb_base.msg import DriveForwardAction, DriveForwardActionGoal, TurnAroundActionGoal, TurnAroundAction


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


if __name__ == '__main__':
    rospy.init_node('TBLogicNode')
    xx = TBLogic()
    xx.drive_forward(38)
    xx.turn_around(90)
    rospy.spin()
