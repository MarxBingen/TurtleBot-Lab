#!/usr/bin/env python

import rospy
import sys
import time
import math
import numpy as np
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Image

from WallDetection import WallDetection
from TBMap import TBMap
from TBPoiDetect import TBPoiDetect

class TBBase:

	movePub = None
	gridSize = 0.5
	speed = 0.2 #0,2 meter pro sek
	turnSpeed = math.radians(45) #45 grad pro sekunde
	heading = 0
	initialIMUset = False
	initialIMU = 0
	initialIMURaw = None
	magSub = None
	wallDetector = None
	map = None

	def __init__(self,gridSize=0.5):
		print "Starte System..."
		print "GridSize=",gridSize
		self.gridSize=gridSize
		rospy.init_node('TurtleBotLab',anonymous = True)
		self.map = TBMap(int(10/gridSize),self.gridSize)
		self.wallDetector = WallDetection()
		self.objectDetector = TBPoiDetect()
		self.magSub = rospy.Subscriber('mobile_base/sensors/imu_data',Imu,queue_size=1,callback=self.imuCallback)
		time.sleep(1)
		self.movePub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)

	def vorwaerts(self):
		print "Fahre vorwaerts"
		twist = Twist()
		twist.linear.x = self.speed
		t_end = time.time() + (self.gridSize / self.speed)
		wc=''
		startHeading = int(self.heading)
		r = rospy.Rate(15)
		while not rospy.is_shutdown() and (time.time()< t_end):
			r.sleep()
			p = self.pruefeFelder()
			if p.mitte=='Belegt':
				print "STOP"
				self.movePub.publish(Twist())
				break
			wc = self.wallDetector.wallGetsCloser()
			if (not wc == ''):
				if wc == 'rechts':
					twist.angular.z=0.5
				if wc == 'links':
					twist.angular.z=-0.5
			else:
				twist.angular.z = 0.0
				#twist.angular.z = self.keepStraight(startHeading)
			self.movePub.publish(twist)
		p = self.pruefeFelder()
		if (p.mitte=='Belegt'):
			print "STOP"
			self.movePub.publish(Twist())
		print "Feldbelegungen"
		print p
		self.map.updatePosition(1)
		self.map.printPosition()
		self.pruefeFelder(True)
		#self.map.updateMap(self.pruefeFelder())

	def drehe(self,richtung='links'):
		z = self.turnSpeed
		w = self.heading
		new_heading = w-90
		print "Drehe ",richtung
		if richtung == 'rechts':
			z = -self.turnSpeed
			new_heading = w+90
		if new_heading < 0:
			new_heading=new_heading+360
		if new_heading > 360:
			new_heading=new_heading-360
		#new_heading = self.korrigiereHeading(new_heading)
		twist = Twist()
		twist.angular.z = z
		turned = False
		while not rospy.is_shutdown():
			if not rospy.is_shutdown():
				self.movePub.publish(twist)
			sh = self.heading
			#print sh, new_heading
			if (int(sh) == int(new_heading)):
				turned = True
				print "Turn Ready"
				break
		#damit stoppt die Drehung sofort
		self.internalZeroAngular()
		self.map.turned(richtung)
		self.pruefeFelder(True)
		#print "Stopped Turning"

	#sorgt dafuer, dass nach einer Drehung wieder geradeaus faehrt
	def internalZeroAngular(self):
		twist = Twist()
		if not rospy.is_shutdown():
			twist.angular.z = 0.1
			for i in range(0,100):
				self.movePub.publish(Twist())
			twist.angular.z = -0.1
			for i in range(0,100):
				self.movePub.publish(Twist())

	def keepStraight(self, sh):
		ah = int(self.heading)
		if ah > sh+3:
			return -0.25
		elif ah < sh-3:
			return +0.25
		return 0.0


	def korrigiereHeading(self,nh):
		sd = 360
		si = -1
		for i in range(0,4):
			s = self.initialIMU + ( 90*i)
			if s >=360:
				s = s - 360
			t = abs(nh - s)
			if (t < sd):
				sd = t
				si = i
		result = self.initialIMU + (90*si)
		if result>=360:
			result=result-360
		return result

	def imuCallback(self,data):
		if rospy.is_shutdown() and not self.magSub is None:
			self.magSub.unregister()
			return
		o = data.orientation
		if (self.initialIMUset==False):
			self.initialIMU=180
			self.initialIMUset=True
			self.initialIMURaw = tf.transformations.quaternion_inverse(np.array([o.x,o.y,o.z,o.w])) 
			return
		current = np.array([o.x,o.y,o.z,o.w])
		correct = tf.transformations.quaternion_multiply(current,self.initialIMURaw)
		qx = correct[0]
		qy = correct[1]
		qz = correct[2]
		qw = correct[3]
		s3 = 2 * ((qw*qz) + (qx*qy))
		s4 = 1 - 2 * ((qy*qy) + (qz*qz))
		#yaw hat nun den Winkel zur Ausgangsposition = 180 Grad
		yaw = 180 - math.degrees(math.atan2(s3,s4))
		self.heading = yaw
		#permanent TF broadcasten
		if not rospy.is_shutdown():
			self.map.broadcastMapToOdomTF(correct)

	def pruefeFelder(self,updateMap = False):
		#w = self.wallDetector.detectWalls2()
		self.wallDetector.lock.acquire()
		w = self.wallDetector.getLastWallInfo()
		self.wallDetector.lock.release()
		if (updateMap):
			print "Aktualisiere Map"
			self.map.updateMap(w)
		return w


	def pruefeObject(self):
		data = rospy.wait_for_message('/camera/rgb/image_color',Image,10.0)
		detected = self.objectDetector.detect(data)
		return detected
