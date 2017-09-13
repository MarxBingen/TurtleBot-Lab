#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid


class TBPathFinding(object):
    '''
    knoten ist eine Liste von Knoten
    kanten ist eine Liste von 3-Tupeln: (knoten1, knoten2, kosten)
    '''

    knoten = None
    kanten = None

    def __init__(self):
        self.knoten = []
        self.kanten = []

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
            return weg
        else:
            raise "Kein Weg gefunden"

    def update_from_map(self):
        '''
        aktualisiert die Knoten und Kanten
        aus dem OccupancyGrid
        '''
        map_data = OccupancyGrid(rospy.wait_for_message('mapLab', OccupancyGrid, 5.0))
        if map_data:
            self.knoten = []
            #TODO knoten und kanten
		


if __name__ == '__main__':
    knoten = []
    knoten.append("1,1")
    knoten.append("2,1")
    knoten.append("3,1")
    knoten.append("4,1")

    knoten.append("1,2")
    knoten.append("4,2")

    knoten.append("1,3")
    knoten.append("3,3")
    knoten.append("4,3")

    knoten.append("1,4")
    knoten.append("2,4")
    knoten.append("3,4")

    knoten.append("3,5")
    knoten.append("4,5")
    knoten.append("5,5")

    kanten = []
    kanten.append(("1,1", "2,1", 1))
    kanten.append(("1,1", "1,2", 1))

    kanten.append(("2,1", "3,1", 1))
    kanten.append(("3,1", "4,1", 1))

    kanten.append(("4,1", "4,2", 1))
    kanten.append(("4,2", "4,3", 1))
    kanten.append(("4,3", "3,3", 1))
    kanten.append(("3,3", "3,4", 1))
    kanten.append(("3,4", "3,5", 1))
    kanten.append(("3,5", "4,5", 1))
    kanten.append(("4,5", "5,5", 1))

    kanten.append(("1,2", "1,3", 1))
    kanten.append(("1,3", "1,4", 1))
    kanten.append(("1,4", "2,4", 1))
    kanten.append(("2,4", "3,4", 1))
    start = "1,1"
    ziel = "5,5"
    test1 = TBPathFinding()
    test1.kanten = kanten
    test1.knoten = knoten
    print test1.dijkstra(start, ziel)
