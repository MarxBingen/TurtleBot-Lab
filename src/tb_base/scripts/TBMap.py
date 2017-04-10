#!/usr/bin/env python

import rospy
import tf_conversions
from tf2_ros import TransformBroadcaster

from TBHeading import SimpleHeading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,Point,TransformStamped,Quaternion
class TBMap:
	
	mapArray = []
	posX=0
	posY=0
	heading = SimpleHeading.NORD
	map = OccupancyGrid()
	mapPub = None

	def __init__(self,size,raster):
		self.size=size
		self.raster=raster
		self.mapArray = [-1]*(size*2*size*2)
		#Ausgangsposition auf null setzen
		self.posX = size
		self.posY = size
		self.mapPub = rospy.Publisher('map',OccupancyGrid,queue_size=1)
		self.tfPub = TransformBroadcaster()

	def turn(self,richtung):
		self.heading = SimpleHeading.turn(self.heading,richtung)
		print self.heading

	def printPosition(self):
		print "Position:", self.posX-self.size, self.posY-self.size
		print "Heading:",self.heading

	def updatePosition(self,step):
		if self.heading is SimpleHeading.NORD:
			self.posY = self.posY + step
		elif self.heading is SimpleHeading.SUED:
			self.posY = self.posY - step
		elif self.heading is SimpleHeading.WEST:
			self.posX = self.posX - step
		elif self.heading is SimpleHeading.OST:
			self.posX = self.posX + step


	def broadcastMapToOdomTF(self):
		#broadcast map to odom transform
		t = TransformStamped()
		t.child_frame_id = "base_footprint"
		t.header.frame_id = "map"
		t.header.stamp = rospy.Time.now()
		t.transform.translation.x=((self.posX-self.size)*self.raster)+(self.raster/2)
		t.transform.translation.y=((self.posY-self.size)*self.raster)+(self.raster/2)
		t.transform.rotation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,SimpleHeading.yaw(self.heading)))
		#t.transform.rotation.w = 1.0 
		if not rospy.is_shutdown():
			self.tfPub.sendTransform(t)

	def updateMap(self, feldbelegung):
		posX=self.posX
		posY=self.posY
		s=self.size*2
		self.mapArray[posY*s+posX]=0
		if self.heading is SimpleHeading.NORD:
			self.mapArray[(posY+1)*s + posX  ] = 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[(posY  )*s + posX-1] = 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[(posY  )*s + posX+1] = 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.SUED:
			self.mapArray[(posY-1 )*s+ posX  ]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[(posY   )*s+ posX+1]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[(posY   )*s+ posX-1]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.WEST:
			self.mapArray[(posY   )*s+ posX-1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[(posY-1 )*s+ posX  ]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[(posY+1 )*s+ posX  ]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.OST:
			self.mapArray[(posY   )*s+ posX+1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[(posY+1 )*s+ posX  ]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[(posY-1 )*s+ posX  ]= 0 if feldbelegung.rechts == 'Frei' else 100
		self.updateOccupancyGrid()

	def updateOccupancyGrid(self):
		self.map.info.resolution=self.raster
		self.map.info.width=self.size*2
		self.map.info.height=self.size*2
		self.map.info.origin=Pose()
		self.map.info.origin.position=Point(-self.size*self.raster,-self.size*self.raster,0)
		self.map.header.frame_id = "map"
		self.map.header.stamp = rospy.Time.now()
		self.map.data=self.mapArray
		if not rospy.is_shutdown():
			self.mapPub.publish(self.map) 
