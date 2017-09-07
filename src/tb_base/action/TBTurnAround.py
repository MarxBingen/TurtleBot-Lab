#!/usr/bin/env python

import rospy
import actionlib
import math
import numpy as np
import tf

from tb_base.msg import TurnAroundAction, TurnAroundFeedback, TurnAroundResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TBTurnAroundServer:

    status = 'Stopped'
    speed = 0
    degrees = 0
    new_heading = 0
    heading = 0
    feedback = TurnAroundFeedback()
    result = TurnAroundResult()

    def __init__(self):
        print "TurnAroundActionServer wird initialisiert"
        self.server = actionlib.SimpleActionServer(
            'TurnAround', TurnAroundAction, auto_start=False)
        # Subscribe to Odom und Bumper
        self.magSub = rospy.Subscriber('odom', Odometry, queue_size=1, callback=self.odomCallback)
        self.movePub = rospy.Publisher(
            'cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        self.server.register_goal_callback(self.new_goal_callback)
        # Server starten
        self.server.start()
        print "TurnAroundActionServer wurde gestartet"

    def odomCallback(self, odom):
        orient = odom.pose.pose.orientation
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        yaw = math.degrees(yaw) + 180
        self.heading = yaw
        if (self.status == 'Turning'):
            self.update()

    def new_goal_callback(self):
        # TODO: wenn noch ein goal active is, dann noch anpassen
        self.server.accept_new_goal()
        g = self.server.current_goal.get_goal()
        self.speed = -g.speed if g.degrees < 0 else g.speed
        self.speed = math.radians(self.speed)
        print "new goal:", self.speed
        self.degrees = g.degrees
        self.new_heading = self.heading + self.degrees
        self.initial_heading = self.heading
        if self.new_heading < 0:
            self.new_heading = self.new_heading + 360
        if self.new_heading > 360:
            self.new_heading = self.new_heading - 360
        self.status = 'Turning'

    def update(self):
        #new_heading = self.korrigiereHeading(new_heading)
        twist = Twist()
        twist.angular.z = self.speed
        self.movePub.publish(twist)
        sh = self.heading
        if int(sh) == int(self.new_heading):
            print "Gedreht"
            self.status = 'Stopped'
            self.result.degrees_turned = abs(self.initial_heading - self.new_heading)
            self.result.new_heading = sh
            self.result.canceled = False
            self.internalZeroAngular()
            self.server.set_succeeded(self.result)

    # sorgt dafuer, dass nach einer Drehung wieder richtig geradeaus faehrt
    def internalZeroAngular(self):
        twist = Twist()
        if not rospy.is_shutdown():
            twist.angular.z = 0.1
            for i in range(0, 100):
                self.movePub.publish(Twist())
            twist.angular.z = -0.1
            for i in range(0, 100):
                self.movePub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('DriveForwardServer')
    server = TBTurnAroundServer()
    rospy.spin()
