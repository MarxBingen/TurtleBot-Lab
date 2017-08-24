#!/usr/bin/env python

import rospy
import actionlib

from tb_base.msg import DriveForwardAction,DriveForwardActionFeedback,DriveForwardActionResult, WallDetection
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

class TBDriveForwardServer:

	initialOdomPosSet = False
	initialOdomPos = None
	status = 'Stopped'
	lastWallDetect = None
	feedback = DriveForwardActionFeedback()
	result = DriveForwardActionResult()

	def __init__(self):
		print "DriveForwardActionServer wird initialisiert"
		self.server = actionlib.SimpleActionServer('DriveForward',DriveForwardAction,self.execute,False)
		#Subscribe to Odom und Bumper
		self.odomSub = rospy.Subscriber('odom',Odometry,queue_size=1,callback=self.odomCallback)
		self.wallSub = rospy.Subscriber('wallDetection',WallDetection,queue_size=1,callback=self.wallCallback)
		self.movePub = rospy.Publisher('cmd_vel_mux/input/safety_controller',Twist, queue_size=1)
		#Server starten
		self.server.start()
		print "DriveForwardActionServer wurde gestartet"

	def odomCallback(self,odom):
		self.lastOdomPos = odom.pose.pose.position

	def wallCallback(self,w):
		self.lastWallDetect = w
		if (self.status == 'Driving'):
			if (self.initialOdomPosSet == False):
				self.initialOdomPos = self.lastOdomPos
				self.initialOdomPosSet = True
			self.update()

	def execute(self, goal):
		self.speed = goal.speed
		self.distance = goal.distance / 100.0
		self.status = 'Driving'
		self.server.set_succeeded()
		
	def update(self):
		twist = Twist()
		twist.linear.x = self.speed
		sop = self.initialOdomPos
		cop = self.lastOdomPos
		odomDiffX = abs(sop.x - cop.x)
		odomDiffY = abs(sop.y - cop.y)
		if odomDiffX > self.distance or odomDiffY > self.distance:
			print "Gefahren"
			self.status = 'Stopped'
			self.initialOdomPosSet = False
			self.result.distance_driven = self.distance * 100
			self.result.canceled = False
			self.server.set_succeeded(self.result)
			return
		w = self.lastWallDetect
		if (not w == None):
			if w.close == 3:
				twist.angular.z=0.5
			elif w.close == 1:
				twist.angular.z=-0.5
			elif w.close == 2:
				self.movePub.publish(Twist())
				print "Stop vor Wand"
				self.status = 'Stopped'
				self.result.distance_driven = dist = math.hypot(odomDiffX, odomDiffY)
				self.result.canceled = True
				self.server.set_aborted(self.result, 'Stop vor Wand')
				return
		else:
			twist.angular.z = 0.0
		self.movePub.publish(twist)
		


if __name__ =='__main__':
	rospy.init_node('DriveForwardServer')
	server = TBDriveForwardServer()
	rospy.spin()
