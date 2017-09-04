#!/usr/bin/env python
"""Map Module"""

import math
import rospy
import tf
import tf2_ros

from TBHeading import SimpleHeading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped, Quaternion
from tb_base.srv import MapDriven, MapTurned, MapTurnedResponse, MapDrivenResponse
from tb_base.msg import WallDetection

class TBMap(object):
    """Class for creating a Map"""
    map_array = []
    pos_x = 0
    pos_y = 0
    heading_simple = SimpleHeading.OST
    map = OccupancyGrid()
    map_pub = None
    pose_pub = None
    tf_pub = None
    lastImu = None
    service_d = None
    service_t = None

    def __init__(self, size, raster):
        self.size = size
        self.raster = raster
        self.map_array = [-1] * (size * 2 * size * 2)
        # Ausgangsposition auf null setzen
        self.pos_x = size
        self.pos_y = size
        # services starten
        self.service_d = rospy.Service('MapServiceDriven', MapDriven, self.driven)
        self.service_t = rospy.Service('MapServiceTurned', MapTurned, self.turned)
        self.map_pub = rospy.Publisher('mapLab', OccupancyGrid, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher('myPose', PoseStamped, queue_size=1)

    def turned(self, request):
        """Callback for MapServiceTurned"""
        self.heading_simple = SimpleHeading.from_degrees(request.new_heading)
        self.updateMap()
        return MapTurnedResponse()

    def driven(self, request):
        """Callback for MapServiceDriven"""
        self.pos_x = int(request.position.x)
        self.pos_y = int(request.position.y)
        self.updateMap()
        return MapDrivenResponse()

    def print_position(self):
        print "Position:", self.pos_x - self.size, self.pos_y - self.size
        print "Heading:", self.heading_simple

    def broadcastMapToOdomTF(self):
        p = PoseStamped()
        p.header.frame_id = "mapLab"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = ((self.pos_x - self.size) *
                             self.raster) + (self.raster / 2)
        p.pose.position.y = ((self.pos_y - self.size) *
                             self.raster) + (self.raster / 2)
        q = tf.transformations.quaternion_from_euler(0,0,SimpleHeading.yaw(self.heading))
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        # broadcast map to odom transform
        t = TransformStamped()
        t.child_frame_id = "base_footprint"
        t.header.frame_id = "mapLab"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = p.pose.position.x
        t.transform.translation.y = p.pose.position.y
        t.transform.rotation = p.pose.orientation
        
        if not rospy.is_shutdown():
            self.pose_pub.publish(p)
            self.tf_pub.sendTransform(t)

    def updateMap(self):
        feldbelegung = rospy.wait_for_message('wallDetection', WallDetection, 2.0)
        pos_x = self.pos_x
        pos_y = self.pos_y
        #update Dijkstra
        #TODO: should not be done here, consider independent node
        #self.update_dijkstra(pos_x, pos_y, feldbelegung)
        # MAP kram
        s = self.size * 2
        self.map_array[pos_y * s + pos_x] = 0
        if self.heading_simple is SimpleHeading.NORD:
            self.map_array[(pos_y + 1) * s + pos_x] = 0 if feldbelegung.mitte == 'Frei' else 100
            self.map_array[(pos_y) * s + pos_x - 1] = 0 if feldbelegung.links == 'Frei' else 100
            self.map_array[(pos_y) * s + pos_x + 1] = 0 if feldbelegung.rechts == 'Frei' else 100
        elif self.heading_simple is SimpleHeading.SUED:
            self.map_array[(pos_y - 1) * s + pos_x] = 0 if feldbelegung.mitte == 'Frei' else 100
            self.map_array[(pos_y) * s + pos_x + 1] = 0 if feldbelegung.links == 'Frei' else 100
            self.map_array[(pos_y) * s + pos_x - 1] = 0 if feldbelegung.rechts == 'Frei' else 100
        elif self.heading_simple is SimpleHeading.WEST:
            self.map_array[(pos_y) * s + pos_x - 1] = 0 if feldbelegung.mitte == 'Frei' else 100
            self.map_array[(pos_y - 1) * s + pos_x] = 0 if feldbelegung.links == 'Frei' else 100
            self.map_array[(pos_y + 1) * s + pos_x] = 0 if feldbelegung.rechts == 'Frei' else 100
        elif self.heading_simple is SimpleHeading.OST:
            self.map_array[(pos_y) * s + pos_x + 1] = 0 if feldbelegung.mitte == 'Frei' else 100
            self.map_array[(pos_y + 1) * s + pos_x] = 0 if feldbelegung.links == 'Frei' else 100
            self.map_array[(pos_y - 1) * s + pos_x] = 0 if feldbelegung.rechts == 'Frei' else 100
        #update map published to ROS
        self.updateOccupancyGrid()

    def updateOccupancyGrid(self):
        self.map.info.resolution = self.raster
        self.map.info.width = self.size * 2
        self.map.info.height = self.size * 2
        self.map.info.origin = Pose()
        self.map.info.origin.position = Point(
            -self.size * self.raster, -self.size * self.raster, 0)
        self.map.header.frame_id = "mapLab"
        self.map.header.stamp = rospy.Time.now()
        self.map.data = self.map_array
        self.map_pub.publish(self.map)

if __name__ == '__main__':
    rospy.init_node('MapService')
    try:
        R = rospy.get_param('~raster')
    except KeyError:
        print "Nutze default Raster 0.38"
        R = 0.38
    try:
        S = rospy.get_param('~size')
    except KeyError:
        S = int(math.ceil(10 / R))
    SERVER = TBMap(S, R)
    rospy.spin()
