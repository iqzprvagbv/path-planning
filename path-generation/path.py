# Contains the implementation of a path which is simply a series of splines
from math import modf
from scipy.integrate import quad
from numpy.linalg import norm

class Path:
    def __init__(self, splines=[]):
        self.splines  = splines
        self.segments = len(splines)

    # Adds a new spline to the end of the path
    # Worth noting this doesn't care if they agree
    # on their boundary, although the rest of this code will
    # expect this to be the case
    def stitch(self,spline):
        self.splines.append(spline)
        self.segments += 1

    # Given a time between 0 and 1 this returns the spline this
    # time lands in and the corresponding time to evaluate. This
    # is basically all the logic as then we can just return the
    # splines data at that point
    def __pick_spline(self,t):
        if t == 1:
            return (self.splines[self.length-1],1)
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
        return s.curvature_radius(t)

    def length(self,a,b):
        return quad(lambda t: norm(self.tangent(t)),a,b)[0]
