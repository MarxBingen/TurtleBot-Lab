#!/usr/bin/env python

import rospy
import sys
import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from WallDetection import WallDetection
from TBMap import TBMap

class TBBase:

	movePub = None
	gridSize = 0.5
	speed = 0.2 #0,2 meter pro sek
	turnSpeed = math.radians(30) #45 grad pro sekunde
	heading = 0
	magSub = None
	wallDetector = None
	map = None

	def __init__(self):
		print "Starte System..."
		rospy.init_node('TurtleBotLab')
		self.map = TBMap(10)
		self.wallDetector = WallDetection()
		self.magSub = rospy.Subscriber('mobile_base/sensors/imu_data',Imu,self.imuCallback)
		time.sleep(1)
		self.movePub = rospy.Publisher('cmd_vel_mux/input/safety_controller',Twist, queue_size=1)

	def vorwaerts(self):
		print "Fahre vorwaerts"
		twist = Twist()
		twist.linear.x = self.speed
		t_end = time.time() + (self.gridSize / self.speed)
		wc=''
		while not rospy.is_shutdown() and (time.time()< t_end):
			if self.pruefeFelder().mitte=='Belegt':
				break
			wc = self.wallDetector.wallGetsCloser()
			if (not wc == ''):
				if wc == 'rechts':
					twist.angular.z=0.58
				if wc == 'links':
					twist.angular.z=-0.58
			else:
				twist.angular.z=0
			self.movePub.publish(twist)
		#self.movePub.publish(Twist())
		self.map.updatePosition()
		self.map.printPosition()

	def drehe(self,richtung='rechts'):
		z = self.turnSpeed
		w = self.heading
		new_heading = self.heading+90
		if richtung == 'links':
			z = -self.turnSpeed
			new_heading = self.heading-90
			self.map.turn('links')
		else:
			self.map.turn('rechts')
		if new_heading <0:
			new_heading=new_heading+360
		if new_heading > 360:
			new_heading=new_heading-360
		twist = Twist()
		twist.angular.z = z
		turned = False
		print "neu:" + str(new_heading)
		print "akt:" + str(self.heading)
		nh = new_heading
		while not rospy.is_shutdown() and (turned == False):
			self.movePub.publish(twist)
			sh = self.heading
			if (int(sh) == int(nh)):
				print "Ready"
				turned = True
		self.movePub.publish(Twist())
		print "Stopped Turning"

	def imuCallback(self,data):
		if rospy.is_shutdown() and not self.magSub is None:
			self.magSub.unregister()
			return
		qx = data.orientation.x
		qy = data.orientation.y
		qz = data.orientation.z
		qw = data.orientation.w
		s3 = 2 * ((qw*qz) + (qx*qy))
		s4 = 1 - 2 * ((qy*qy) + (qz*qz))
		yaw = math.degrees(math.atan2(s3,s4))+180
		self.heading = yaw

	def pruefeFelder(self):
		return self.wallDetector.detectWalls()
