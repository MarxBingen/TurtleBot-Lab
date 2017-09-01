#!/usr/bin/env python
"""Pathfinding Module"""

import math
import rospy

from TBHeading import SimpleHeading


class PathFinder(object):

    knoten = []
    kanten = []


    def update_dijkstra(self, px, py, heading, feldbelegung):
        """refreshes nodes and edges"""
        cKnot = str(px, ",", py)
        mknot = ""
        lknot = ""
        rknot = ""
        if heading is SimpleHeading.NORD:
            mknot = str(py + 1) + "," + str(px)
            lknot = str(py) + "," + str(px - 1)
            rknot = str(py) + "," + str(px + 1)
        elif heading is SimpleHeading.SUED:
            mknot = str(py - 1) + "," + str(px)
            lknot = str(py) + "," + str(px + 1)
            rknot = str(py) + "," + str(px - 1)
        elif heading is SimpleHeading.WEST:
            mknot = str(py) + "," + str(px - 1)
            lknot = str(py - 1) + "," + str(px)
            rknot = str(py + 1) + "," + str(px)
        elif heading is SimpleHeading.OST:
            mknot = str(py) + "," + str(px + 1)
            lknot = str(py + 1) + "," + str(px)
            rknot = str(py - 1) + "," + str(px)
        # knoten und kanten aktualiseiren
        if lknot not in self.knoten:
            self.knoten.append(lknot)
        if mknot not in self.knoten:
            self.knoten.append(mknot)
        if rknot not in self.knoten:
            self.knoten.append(rknot)
        if feldbelegung.mitte == 'Frei':
            if (cKnot, mknot, 1) not in self.kanten:
                self.kanten.append((cKnot, mknot, 1))
        else:
            if (cKnot, mknot, 1) in self.kanten:
                self.kanten.remove((cKnot, mknot, 1))
        if feldbelegung.rechts == 'Frei':
            if (cKnot, rknot, 1) not in self.kanten:
                self.kanten.append((cKnot, rknot, 1))
        else:
            if (cKnot, rknot, 1) in self.kanten:
                self.kanten.remove((cKnot, rknot, 1))
        if feldbelegung.links == 'Frei':
            if (cKnot, lknot, 1) not in self.kanten:
                self.kanten.append((cKnot, lknot, 1))
        else:
            if (cKnot, lknot, 1) in self.kanten:
                self.kanten.remove((cKnot, lknot, 1))
