#!/usr/bin/env python

import rospy
import sys
import time
import math
import numpy

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
	initialIMUset = False
	initialIMU = 0
	magSub = None
	wallDetector = None
	map = None

	def __init__(self,gridSize=0.5):
		print "Starte System..."
		print "GridSize=",gridSize
		self.gridSize=gridSize
		rospy.init_node('TurtleBotLab')
		self.map = TBMap(10,self.gridSize)
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
					twist.angular.z=0.5
				if wc == 'links':
					twist.angular.z=-0.5
			else:
				twist.angular.z=0
			self.movePub.publish(twist)
		#self.movePub.publish(Twist())
		self.map.updatePosition(1)
		self.map.printPosition()
		#self.map.updateMap(self.pruefeFelder())

	def drehe(self,richtung='links'):
		z = self.turnSpeed
		w = self.heading
		new_heading = self.heading+90
		if richtung == 'rechts':
			print "Drehe Rechts"
			z = -self.turnSpeed
			new_heading = self.heading-90
			self.map.turn('rechts')
		else:
			print "Drehe Links"
			self.map.turn('links')
		if new_heading <0:
			new_heading=new_heading+360
		if new_heading > 360:
			new_heading=new_heading-360
		print "Old:", new_heading
		new_heading = self.korrigiereHeading(new_heading)
		print "New:", new_heading
		twist = Twist()
		twist.angular.z = z
		turned = False
		while not rospy.is_shutdown() and (turned == False):
			self.movePub.publish(twist)
			sh = self.heading
			if (int(sh) == int(new_heading)):
				turned = True
				print "Turn Ready"
		if not rospy.is_shutdown():
			self.movePub.publish(Twist())
		#print "Stopped Turning"

	def korrigiereHeading(self,nh):
		sd = 360
		si = -1
		print "initIMU", self.initialIMU
		for i in range(0,4):
			t = abs(nh - (self.initialIMU + (90*i)))
			print i, t, sd, si
			if (t < sd):
				sd = t
				si = i
		return self.initialIMU + (90*si)

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
		if (self.initialIMUset==False):
			self.initialIMU=yaw
			self.initialIMUset=True
		#permanent TF broadcasten
		self.map.broadcastMapToOdomTF()

	def pruefeFelder(self):
		w = self.wallDetector.detectWalls()
		self.map.updateMap(w)
		return w
