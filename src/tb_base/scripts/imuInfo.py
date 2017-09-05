#!/usr/bin/env python

import rospy
import sys
sys.path.insert(0, '../srv')
import time
import math
import numpy as np
import tf

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from TBHeading import SimpleHeading

initial = None
initialSet = False


def imuCallback(data):
    global initialSet, initial
    if initialSet == False:
        o = data.orientation
        initial = tf.transformations.quaternion_inverse(
            np.array([o.x, o.y, o.z, o.w]))
        initialSet = True
        return
    #test = data.orientation * initial.orientation
    o = data.orientation
    current = np.array([o.x, o.y, o.z, o.w])
    new = tf.transformations.quaternion_multiply(current, initial)
    #,initial.orientation)
    qx = new[0]
    qy = new[1]
    qz = new[2]
    qw = new[3]
    s3 = 2 * ((qw * qz) + (qx * qy))
    s4 = 1 - (2 * ((qy * qy) + (qz * qz)))
    yaw = 180 - (math.degrees(math.atan2(s3, s4)))
    print qz, qw, yaw


def odomCallback(data):
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    print y * (180/math.pi)
    heading_simple = SimpleHeading.from_quaternion(data.pose.pose.orientation)
    h2 = SimpleHeading.yaw(heading_simple)
    print heading_simple, h2, y


if __name__ == '__main__':
    rospy.init_node('ImuReader')
    #magSub = rospy.Subscriber('mobile_base/sensors/imu_data',Imu,imuCallback)
    magSub = rospy.Subscriber('odom', Odometry, odomCallback)
    rospy.spin()
