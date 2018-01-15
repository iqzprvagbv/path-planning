# Contains the implementation of a path which is simply a series of splines
import spline as s
from math import modf
from scipy.integrate import quad
from numpy.linalg import norm
from scipy.optimize import newton

class Path:
    def __init__(self, splines=[]):
        self.splines  = splines
        self.segments = len(splines)
        self.l = self.length(0,1)
    # Adds a new spline to the end of the path
    # Worth noting this doesn't care if they agree
    # on their boundary, although the rest of this code will
    # expect this to be the case
    def stitch(self,spline):
        self.splines.append(spline)
        self.segments += 1
        self.l = self.length(0,1)

    # Given a time between 0 and 1 this returns the spline this
    # time lands in and the corresponding time to evaluate. This
    # is basically all the logic as then we can just return the
    # splines data at that point
    def __pick_spline(self,t):
        if t == 1:
            return (self.splines[self.segments-1],1)
        x, s = modf(t*self.segments)
        return (self.splines[int(s)],x)

    def eval(self,t):
        s, x = self.__pick_spline(t)
        return s.eval(x)

    def tangent(self,t):
        s, x = self.__pick_spline(t)
        return self.segments * s.tangent(x)

    def curvature_radius(self,t):
        s, x = self.__pick_spline(t)
        return s.curvature_radius(x)

    def length(self,a,b):
        return quad(lambda t: norm(self.tangent(t)),a,b)[0]

    def __next(self, t, ds, guess):
        if self.length(0,t) + ds > self.l:
            root =  1
        else:
            f = lambda x: self.length(t,x) - ds
            fprime = lambda x: norm(self.tangent(t))
            root = newton(f, guess, fprime)

        return root

#    def planning_times(self,ds):
#        for s in self.splines:
#            for t in s.planning_times:
#                yield t/float(self.segments)

    def planning_times(self,ds):
        t  = 0
        last_t = None
        dt = 0
        while t < 1:
            yield t
            last_t = t
            t = self.__next(t,ds,t+dt)
            dt = t - last_t
        yield 1

    def draw(self,plt,segmented=False):
        if segmented:
            for s in self.splines:
                s.draw(plt)
        else:
            points = []
            for t in range(1000):
                points.append(self.eval(t/1000.0))
            points.append(self.eval(1))
            x,y = zip(*points)
            plt.plot(x,y)

def from_waypoints(waypoints):
    last_waypoint = None
    splines = []
    for w in waypoints:
        if not last_waypoint:
            last_waypoint = w
        else:
            spline = s.from_waypoints(last_waypoint,w)
            splines.append(spline)
            last_waypoint = w
    return Path(splines)
