#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from tb_base.srv import PathFind, PathFindResponse, MapInfo,MapInfoRequest


class TBPathFind(object):
    '''
    knoten ist eine Liste von Knoten
    kanten ist eine Liste von 3-Tupeln: (knoten1, knoten2, kosten)
    '''

    knoten = None
    kanten = None

    def __init__(self):
        self.knoten = []
        self.kanten = []
        self.s = rospy.Service('PathFind', PathFind, self.findpath)
        print "Verbinde mit MapServiceInfo..."
        rospy.wait_for_service('MapServiceInfo')
        self.mapServiceInfo = rospy.ServiceProxy('MapServiceInfo', MapInfo)
        print "PathFindService gestartet"

    def findpath(self, request):
        print request
        #zuerst knoten und kanten aktualisieren
        self.update_from_map()
        result = PathFindResponse()
        sk = (request.start.x,request.start.y)
        zk = (request.ziel.x,request.ziel.y)
        result.path = self.dijkstra(sk,zk)
        return result

    def dijkstra(self, start, ziel):
        ''' knoten ist eine Liste von Knoten
         kanten ist eine Liste von 3-Tupeln: (knoten1, knoten2, kosten)
         start ist der Knoten, in dem die Suche startet
         ziel ist der Knoten, zu dem ein Weg gesucht werden soll
         Gibt ein Tupel zurueck mit dem Weg und den Kosten
        '''
		#knoten_eigenschaften = knoten, kosten,temp,besucht
        knoten_eigenschaften = [[i, "inf", None, False]
                                for i in self.knoten if i != start]
        knoten_eigenschaften += [[start, 0, None, False]]
        for i, item in enumerate(knoten_eigenschaften):
            item += [i]
        while True:
            unbesuchte_knoten = [x for x in knoten_eigenschaften if not x[3]]
            if not unbesuchte_knoten:
                break
            sortierte_liste = sorted(unbesuchte_knoten, key=lambda i: i[1])
            aktiver_knoten = sortierte_liste[0]
            knoten_eigenschaften[aktiver_knoten[4]][3] = True
            if aktiver_knoten[0] == ziel:
                break
            aktive_kanten = [
                x for x in self.kanten if x[0] == aktiver_knoten[0]]
            for kante in aktive_kanten:
                anderer_knoten_id = [
                    x for x in knoten_eigenschaften if x[0] == kante[1]][0][4]
                gewicht_summe = aktiver_knoten[1] + kante[2]
                if gewicht_summe < knoten_eigenschaften[anderer_knoten_id][1]:
                    knoten_eigenschaften[anderer_knoten_id][1] = gewicht_summe
                    knoten_eigenschaften[anderer_knoten_id][2] = aktiver_knoten[4]

        if aktiver_knoten[0] == ziel:
            weg = []
            weg += [aktiver_knoten[0]]
            kosten = 0
            while aktiver_knoten[0] != start:
                aktiver_knoten = knoten_eigenschaften[aktiver_knoten[2]]
                weg += [aktiver_knoten[0]]
                kosten += aktiver_knoten[1]
            weg.reverse()
            return [Point(k[0],k[1],0) for k in weg]
        else:
            rospy.loginfo("Kein Weg gefunden")
            return []

    def update_from_map(self):
        '''
        aktualisiert die Knoten und Kanten
        aus dem OccupancyGrid
        '''
        response = self.mapServiceInfo(MapInfoRequest())
        map_data = response.map
        kantencheck = self._is_kante
        if map_data:
            self.knoten = []
            for i, item in enumerate(map_data.data):
                if item == 0:
                    x = i % map_data.info.width
                    x = x - (map_data.info.width / 2)
                    y = int(i / map_data.info.width)
                    y = y - (map_data.info.height / 2)
                    self.knoten.append((x,y))
            self.kanten = [ (k1,k2,1) for k1 in self.knoten for k2 in self.knoten if kantencheck(k1,k2) ]
            #TODO  und kanten
    
    
    def _is_kante(self,knoten1,knoten2):
            diff = (abs(knoten1[0]-knoten2[0]),abs(knoten1[1]-knoten2[1]))
            diff_ok = (diff[0]==0 or diff[0]==1) and (diff[1]==0 or diff[1]==1)
            return diff_ok and (diff[0]!=diff[1])

if __name__ == '__main__':
    rospy.init_node('TBPathFindService')
    p = TBPathFind()
    rospy.spin()
