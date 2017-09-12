#!/usr/bin/env python

import rospy
import actionlib
import math

from tb_base.msg import DriveForwardAction, DriveForwardFeedback, DriveForwardResult, WallDetection
from tb_base.srv import MapDriven
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class TBDriveForwardServer:
    '''Implementiert einen ActionServer, Verwendung jedoch nicht vollstaendig wie
    erwartet
    '''

    initialOdomPosSet = False
    initialOdomPos = None
    lastOdomPos = None
    status = 'Stopped'
    lastWallDetect = None
    feedback = DriveForwardFeedback()
    result = DriveForwardResult()
    mapServiceD = None
    speed = 0.0
    distance = 0

    def __init__(self):
        print "DriveForwardActionServer wird initialisiert"
        self.server = actionlib.SimpleActionServer('DriveForward',DriveForwardAction,auto_start=False)
        # Subscribe to Odom und Bumper
        self.odomSub = rospy.Subscriber('odom', Odometry, queue_size=1, callback=self.odomCallback)
        self.wallSub = rospy.Subscriber('wallDetection', WallDetection, queue_size=1, callback=self.wallCallback)
        self.movePub = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        #MapService verknuepfen
        print "Verbinde mit MapService..."
        rospy.wait_for_service('MapServiceDriven')
        self.mapServiceD = rospy.ServiceProxy('MapServiceDriven', MapDriven)
        print "MapService gefunden"
        #new goal callback registrieren
        self.server.register_goal_callback(self.new_goal_callback)
        # Server starten
        self.server.start()
        print "DriveForwardActionServer wurde gestartet"

    def odomCallback(self, odom):
        self.lastOdomPos = odom
        
    def wallCallback(self, w):
        self.lastWallDetect = w
        if not self.status == 'Stopped':
            if (self.initialOdomPosSet == False):
                self.initialOdomPos = self.lastOdomPos
                self.initialOdomPosSet = True
            self.update()

    def new_goal_callback(self):
        g = self.server.accept_new_goal()
        print g
        self.speed = g.speed
        self.distance = g.distance / 100.0
        if g.stop == True:
            self.status = 'Stopped'
            self.movePub.publish(Twist())
            self.mapServiceD(self.lastOdomPos)
            self.result = DriveForwardResult()
            self.result.distance_driven = self.distance * 100
            self.result.canceled = False
            self.result.position = self.lastOdomPos.pose.pose.position
            self.server.set_aborted(self.result)
            print "Stopped by Command"
        else:
            self.initialOdomPosSet = False
            self.status = 'Driving'

    def update(self):
        twist = Twist()
        twist.linear.x = self.speed
        sop = self.initialOdomPos
        cop = self.lastOdomPos
        odomDiffX = abs(sop.pose.pose.position.x - cop.pose.pose.position.x)
        odomDiffY = abs(sop.pose.pose.position.y - cop.pose.pose.position.y)
        if self.status == 'Driving' and (odomDiffX > self.distance or odomDiffY > self.distance):
            print "Gefahren"
            self.status = 'Done'
            self.mapServiceD(cop)
            self.initialOdomPosSet = False
            self.result = DriveForwardResult()
            self.result.distance_driven = self.distance * 100
            self.result.canceled = False
            self.result.position = cop.pose.pose.position
            self.server.set_succeeded(self.result)
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
                self.result = DriveForwardResult()
                self.result.distance_driven = math.hypot(odomDiffX, odomDiffY)*100
                self.result.position = cop.pose.pose.position
                self.mapServiceD(cop)
                self.result.canceled = True
                self.server.set_aborted(self.result)
        else:
            twist.angular.z = 0.0
        if not self.status == 'Stopped':
            print "Twisted..."
            self.movePub.publish(twist)
        else:
            self.movePub.publish(Twist())

if __name__ == '__main__':
    rospy.init_node('DriveForwardServer')
    SERVER = TBDriveForwardServer()
    rospy.spin()
