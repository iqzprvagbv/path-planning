# Contains the implementation of a path which is simply a series of splines
from math            import modf

from numpy.linalg    import norm
from scipy.optimize  import brentq
from scipy.integrate import quad

import spline as s

class Path(object):
    def __init__(self, splines):
        self.splines = splines
        self.segments = len(splines)
        self.total_length = self.length(0, 1)
    # Adds a new spline to the end of the path
    # Worth noting this doesn't care if they agree
    # on their boundary, although the rest of this code will
    # expect this to be the case
    def stitch(self, spline):
        self.splines.append(spline)
        self.segments += 1
        self.total_length = self.length(0, 1)

    # Given a time between 0 and 1 this returns the spline this
    # time lands in and the corresponding time to evaluate. This
    # is basically all the logic as then we can just return the
    # splines data at that point
    def __pick_spline(self, t):
        if t >= 1:
            t -= self.segments - 1
            spline = self.segments-1
        elif t <= 0:
            spline = 0
        else:
            x, spline = modf(t*self.segments)
        return (self.splines[int(spline)], x)

    def eval(self, t):
        spline, t = self.__pick_spline(t)
        return spline.eval(t)

    def tangent(self, t):
        spline, t = self.__pick_spline(t)
        return self.segments * spline.tangent(t)

    def unit_tangent(self, t):
        spline, t = self.__pick_spline(t)
        return spline.unit_tangent(t)

    def unit_normal(self, t):
        spline, t = self.__pick_spline(t)
        return spline.unit_normal(t)

    def heading(self, t):
        spline, t = self.__pick_spline(t)
        return spline.heading(t)

    def curvature_radius(self, t):
        spline, t = self.__pick_spline(t)
        return spline.curvature_radius(t)

    def length(self, start, end):
        return quad(lambda t: norm(self.tangent(t)), start, end)[0]

    def __next(self, t, distance):
        if self.length(0, t) + distance > self.total_length:
            root = 1
        else:
            f = lambda x: self.length(t, x) - distance
            root = brentq(f, t, 1)

        return root

    def planning_times(self, distance):
        t = 0
        while t < 1:
            yield t
            t = self.__next(t, distance)
        yield 1

def from_waypoints(waypoints):
    if len(waypoints) < 2:
        return None

    last_waypoint = None
    splines = []
    for waypoint in waypoints:
        if not last_waypoint:
            last_waypoint = waypoint
        else:
            spline = s.from_waypoints(last_waypoint, waypoint)
            splines.append(spline)
            last_waypoint = waypoint
    return Path(splines)
