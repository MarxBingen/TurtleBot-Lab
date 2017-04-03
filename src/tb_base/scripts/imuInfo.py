#!/usr/bin/env python

import rospy
import sys
import time
import math

from sensor_msgs.msg import Imu

def imuCallback(data):
	qx = data.orientation.x
	qy = data.orientation.y
	qz = data.orientation.z
	qw = data.orientation.w
	s3 = 2 * ((qw*qz) + (qx*qy))
	s4 = 1 - 2 * ((qy*qy) + (qz*qz))
	yaw = math.degrees(math.atan2(s3,s4))+180
	print yaw


if __name__ == '__main__':
	rospy.init_node('ImuReader')
	magSub = rospy.Subscriber('mobile_base/sensors/imu_data',Imu,imuCallback)
	rospy.spin()

