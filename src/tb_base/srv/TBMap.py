#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros

from TBHeading import SimpleHeading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped,Pose,Point,TransformStamped,Quaternion
from tb_base.srv import MapDriven, MapTurned, MapTurnedResponse, MapDrivenResponse

class TBMap:

	mapArray = []
	posX=0
	posY=0
	heading = SimpleHeading.OST
	map = OccupancyGrid()
	mapPub = None
	lastImu = None
	knoten = []
	kanten = []

	def __init__(self,size,raster):
		self.size=size
		self.raster=raster
		self.mapArray = [-1]*(size*2*size*2)
		#Ausgangsposition auf null setzen
		self.posX = size
		self.posY = size
		#services starten
		self.sd = rospy.Service('MapServiceDriven', MapDriven, self.driven)
		self.st = rospy.Service('MapServiceTurned', MapTurned, self.turned)
		self.mapPub = rospy.Publisher('mapLab',OccupancyGrid,queue_size=1)
		self.tfPub = tf2_ros.TransformBroadcaster()
		self.posePub = rospy.Publisher('myPose',PoseStamped,queue_size=1)


	def turned(self,request):
		#self.heading = SimpleHeading.turn(self.heading,richtung)
		print self.request
		return MapTurnedResponse()
		
	def driven(self,request):
		print request
		return MapDrivenResponse()

	def printPosition(self):
		#pass
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

	def broadcastMapToOdomTF(self, orientation):
		p = PoseStamped()
		p.header.frame_id = "mapLab"
		p.header.stamp = rospy.Time.now()
		p.pose.position.x=((self.posX-self.size)*self.raster)+(self.raster/2)
		p.pose.position.y=((self.posY-self.size)*self.raster)+(self.raster/2)
		p.pose.orientation.x = orientation[0]
		p.pose.orientation.y = orientation[1]
		p.pose.orientation.z = orientation[2]
		p.pose.orientation.w = orientation[3]
		#broadcast map to odom transform
		t = TransformStamped()
		t.child_frame_id = "base_footprint"
		t.header.frame_id = "mapLab"
		t.header.stamp = rospy.Time.now()
		t.transform.translation.x=p.pose.position.x
		t.transform.translation.y=p.pose.position.y
		t.transform.rotation = p.pose.orientation
		#q = tf.transformations.quaternion_from_euler(0,0,SimpleHeading.yaw(self.heading))
		#t.transform.rotation.x=q[0]
		#t.transform.rotation.y=q[1]
		#t.transform.rotation.z=q[2]
		#t.transform.rotation.w=q[3]
		if not rospy.is_shutdown():
			self.posePub.publish(p)
			self.tfPub.sendTransform(t)

	def updateMap(self, feldbelegung):
		posX=self.posX
		posY=self.posY
		self.updateDijkstraStuff(posX,posY,feldbelegung)
		#MAP kram
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

	def updateDijkstraStuff(self,px,py,feldbelegung):
		cKnot = str(px) + "," + str(py)
		lknot = ""
		mknot = ""
		rknot = ""
		#temp bezeichner erstellen
		if self.heading is SimpleHeading.NORD:
			mknot = str(py+1) + "," + str(px)
			lknot = str(py)   + "," + str(px-1)
			rknot = str(py)   + "," + str(px+1)
		elif self.heading is SimpleHeading.SUED:
			mknot = str(py-1) + "," + str(px)
			lknot = str(py)   + "," + str(px+1)
			rknot = str(py)   + "," + str(px-1)
		elif self.heading is SimpleHeading.WEST:
			mknot = str(py)   + "," + str(px-1)
			lknot = str(py-1) + "," + str(px)
			rknot = str(py+1) + "," + str(px)
		elif self.heading is SimpleHeading.OST:
			mknot = str(py)   + "," + str(px+1)
			lknot = str(py+1) + "," + str(px)
			rknot = str(py-1) + "," + str(px)
		#knoten und kanten aktualiseiren
		if not lknot in self.knoten:
				self.knoten.append(lknot)
		if not mknot in self.knoten:
				self.knoten.append(mknot)
		if not rknot in self.knoten:
				self.knoten.append(rknot)
		if feldbelegung.mitte == 'Frei':
			if not (cKnot,mknot,1) in self.kanten:
				self.kanten.append((cKnot,mknot,1))
		else:
			if (cKnot,mknot,1) in self.kanten:
				self.kanten.remove((cKnot,mknot,1))
		if feldbelegung.rechts == 'Frei':
			if not (cKnot,rknot,1) in self.kanten:
				self.kanten.append((cKnot,rknot,1))
		else:
			if (cKnot,rknot,1) in self.kanten:
				self.kanten.remove((cKnot,rknot,1))
		if feldbelegung.links == 'Frei':
			if not (cKnot,lknot,1) in self.kanten:
				self.kanten.append((cKnot,lknot,1))
		else:
			if (cKnot,lknot,1) in self.kanten:
				self.kanten.remove((cKnot,lknot,1))


	def updateOccupancyGrid(self):
		self.map.info.resolution=self.raster
		self.map.info.width=self.size*2
		self.map.info.height=self.size*2
		self.map.info.origin=Pose()
		self.map.info.origin.position=Point(-self.size*self.raster,-self.size*self.raster,0)
		self.map.header.frame_id = "mapLab"
		self.map.header.stamp = rospy.Time.now()
		self.map.data=self.mapArray
		if not rospy.is_shutdown():
			self.mapPub.publish(self.map)
			
if __name__ =='__main__':
	rospy.init_node('MapService')
	try:
		r = rospy.get_param('~raster')
	except:
		print "Nutze default Raster 0.38"
		r = 0.38
	try:
		s = rospy.get_param('~size')
	except:
		s = int(math.ceil(10/r))
	server = TBMap(s,r)
	rospy.spin()
