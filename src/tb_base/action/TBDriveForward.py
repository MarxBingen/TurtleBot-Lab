#!/usr/bin/env python

import rospy
import actionlib
import math

from tb_base.msg import DriveForwardAction, DriveForwardFeedback, DriveForwardResult, WallDetection
from tb_base.srv import MapDriven
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


class TBDriveForwardServer:

    initialOdomPosSet = False
    initialOdomPos = None
    status = 'Stopped'
    lastWallDetect = None
    feedback = DriveForwardFeedback()
    result = DriveForwardResult()
    mapServiceD = None

    def __init__(self):
        print "DriveForwardActionServer wird initialisiert"
        self.server = actionlib.SimpleActionServer(
            'DriveForward', DriveForwardAction, auto_start=False)
        # Subscribe to Odom und Bumper
        self.odomSub = rospy.Subscriber('odom', Odometry, queue_size=1, callback=self.odomCallback)
        self.wallSub = rospy.Subscriber('wallDetection', WallDetection, queue_size=1, callback=self.wallCallback)
        self.movePub = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        #MapService verknuepfen
        print "Verbinde mit MapService..."
        rospy.wait_for_service('MapServiceDriven')
        self.mapServiceD = rospy.ServiceProxy('MapServiceDriven', MapDriven)
        print "MapService gefunden"
        #erst jetzt fuer Goals registrieren
        self.server.register_goal_callback(self.new_goal_callback)
        # Server starten
        self.server.start()
        print "DriveForwardActionServer wurde gestartet"

    def odomCallback(self, odom):
        self.lastOdomPos = odom

    def wallCallback(self, w):
        self.lastWallDetect = w
        if (self.status == 'Driving'):
            if (self.initialOdomPosSet == False):
                self.initialOdomPos = self.lastOdomPos
                self.initialOdomPosSet = True
            self.update()

    def new_goal_callback(self):
        # TODO: wenn noch ein goal active is, dann noch anpassen
        self.server.accept_new_goal()
        g = self.server.current_goal.get_goal()
        self.speed = g.speed
        self.distance = g.distance / 100.0
        self.status = 'Driving'

    def update(self):
        twist = Twist()
        twist.linear.x = self.speed
        sop = self.initialOdomPos
        cop = self.lastOdomPos
        odomDiffX = abs(sop.pose.pose.position.x - cop.pose.pose.position.x)
        odomDiffY = abs(sop.pose.pose.position.y - cop.pose.pose.position.y)
        if odomDiffX > self.distance or odomDiffY > self.distance:
            print "Gefahren"
            self.status = 'Stopped'
            self.initialOdomPosSet = False
            self.result = DriveForwardResult()
            self.result.distance_driven = self.distance * 100
            self.result.canceled = False
            self.result.position = cop.pose.pose.position
            self.mapServiceD(cop)
            self.server.set_succeeded(self.result)
            return
        w = self.lastWallDetect
        if (not w == None):
            if w.close == 3:
                twist.angular.z = 0.5
            elif w.close == 1:
                twist.angular.z = -0.5
            elif w.close == 2:
                self.movePub.publish(Twist())
                print "Stop vor Wand"
                self.status = 'Stopped'
                self.result.distance_driven = math.hypot(odomDiffX, odomDiffY)*100
                self.result.position = cop.pose.pose.position
                self.mapServiceD(cop)
                self.result.canceled = True
                self.server.set_aborted(self.result, 'Stop vor Wand')
                return
        else:
            twist.angular.z = 0.0
        self.movePub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('DriveForwardServer')
    SERVER = TBDriveForwardServer()
    rospy.spin()
