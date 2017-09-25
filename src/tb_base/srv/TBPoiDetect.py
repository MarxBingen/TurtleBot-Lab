#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tb_base.srv import PoiDetect, PoiDetectResponse, MapPos, PoiDetectAll, PoiDetectAllResponse
from std_srvs.srv import Empty
from tb_base.msg import SimplePosition, PoiInfo


class TBPoiDetectService:

    detections = []

    def __init__(self):
        print "PoiDetectService wird initialisiert"
        self.bridge = CvBridge()
        # Color range definition
        self.redLow = np.array([47, 0, 0], dtype=np.uint8)
        self.redHigh = np.array([255, 16, 14], dtype=np.uint8)
        self.greenLow = np.array([0, 37, 0], dtype=np.uint8)
        self.greenHigh = np.array([22, 255, 42], dtype=np.uint8)
        rospy.wait_for_service('MapServicePos')
        self.mapServicePos = rospy.ServiceProxy('MapServicePos', MapPos)
        # Service starten
        self.s = rospy.Service('PoiDetectService', PoiDetect, self.detect)
        self.all_service = rospy.Service('PoiDetectServiceAllPois', PoiDetectAll, self.allpois)
        self.reset_service = rospy.Service('PoiDetectServiceReset', Empty, self.reset)
        print "PoiDetectService gestartet"

    def allpois(self, req):
        result = PoiDetectAllResponse()
        result.all_pois = self.detections
        return result

    def reset(self, req):
        self.detections = []
        print "Bisher erkannte Objekte geloescht"

    def detect(self, req):
        try:
            data = rospy.wait_for_message(
                '/camera/rgb/image_color', Image, 10.0)
            imgO = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as er:
            print(er)
            result = PoiDetectResponse()
            result.detected = False
            result.color = 'Error'
            return result
        imgC = cv2.cvtColor(imgO, cv2.COLOR_BGR2RGB)
        # bild optimierungen
        imgC = cv2.erode(imgC, None, iterations=2)
        imgC = cv2.dilate(imgC, None, iterations=2)
        # red green maskierungen
        maskRed = cv2.inRange(imgC, self.redLow, self.redHigh)
        maskGreen = cv2.inRange(imgC, self.greenLow, self.greenHigh)
        # Konturen finden
        cntsGreen = cv2.findContours(
            maskGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cntsRed = cv2.findContours(
            maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        detectedColor = None
        if len(cntsGreen) > 0:
            # die groeste kontur finden
            c = max(cntsGreen, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 70 and radius < 150:
                detectedColor = "Green"
        if len(cntsRed) > 0:
            c = max(cntsRed, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 70 and radius < 150:
                detectedColor = "Red"
        # wenn was erkannt wurde, merken
        if detectedColor != None:
            posdata = self.mapServicePos(Empty())
            pi = PoiInfo()
            pi.position = posdata
            pi.color = detectedColor
            self.detections.append(pi)
        result = PoiDetectResponse()
        result.detected = detectedColor != None
        result.color = detectedColor
        print result
        return result


if __name__ == '__main__':

    rospy.init_node('TBPoiDetectService')
    p = TBPoiDetectService()
    rospy.spin()
