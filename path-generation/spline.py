# This file defines the underlying spline we use to represent the paths
# They are quintic bezier curves

from numpy           import dot, array, rot90
from numpy.linalg    import norm
from scipy.integrate import quad
from scipy.optimize  import brentq

class Spline:
    def __init__(self,p0,p1,p2,p3,p4,p5):
        # C is the coeffecient matrix for our spline
        # The polynomial looks like:
        # c[0] + c[1]t + c[2]t^2 + c[3]t^3 + ...
        self.c = []
        self.c.append(p0)
        self.c.append( -5 * p0 +  5 * p1)
        self.c.append( 10 * p0 - 20 * p1 + 10 * p2)
        self.c.append(-10 * p0 + 30 * p1 - 30 * p2 + 10 * p3)
        self.c.append(  5 * p0 - 20 * p1 + 30 * p2 - 20 * p3 + 5 * p4)
        self.c.append( -1 * p0 +  5 * p1 - 10 * p2 + 10 * p3 - 5 * p4 + p5)

        self.l = self.length(0,1)

        self.__last_dt = 0 #This feels messy... only one function really uses it

    # The parameter functions when dotted with our coeffecient matrix
    # will produce the 0th, 1st, and 2nd derivative respectively
    def __parameter(self, t):
        return [1,t,t**2,t**3,t**4,t**5]

    def __dparameter(self, t):
        return [0,1,2*t,3*t**2,4*t**3,5*t**4]

    def __ddparameter(self,t):
        return [0,0,2,6*t,12*t**2,20*t**3]

    def eval(self, t):
        return dot(self.__parameter(t),self.c)

    def __derivative(self, t):
        return dot(self.__dparameter(t),self.c)

    def __double_derivative(self,t):
        return dot(self.__ddparameter(t),self.c)

    def __curvature(self,t):
        t1 = self.__derivative(t)[0] * self.__double_derivative(t)[1]
        t2 = self.__derivative(t)[1] * self.__double_derivative(t)[0]
        t3 = (self.__derivative(t)[0] ** 2) + (self.__derivative(t)[1] ** 2)
        return (t1 - t2)/(t3 ** 1.5)

    def tangent(self,t):
        return self.__derivative(t)

    def unit_tangent(self,t):
        return self.tangent(t)/norm(self.tangent(t))

    def unit_normal(self,t):
        tangent = self.unit_tangent(t)
        return dot(tangent,[[0,1],[-1,0]])

    def curvature_radius(self,t):
        return 1/self.__curvature(t)

    def length(self,a,b):
        return quad(lambda t: norm(self.tangent(t)),a,b)[0]

    # Given a time t and a distance ds this function approximates
    # an s such that arc length from t to s is ds. Guess is a
    # provided approximation of s for the algorithm to start with
    def __next(self, t, ds, guess):
        if self.length(0,t) + ds > self.l:
            root =  1
        else:
            f = lambda x: self.length(t,x) - ds
            fprime = lambda x: norm(self.tangent(t))
            root = brentq(f, t, 1)

        return root

    # Generates a list of t such that the archlength between any two of
    # them are equal.
    def planning_times(self,ds):
        t  = 0
        last_t = None
        dt = 0
        while t < 1:
            yield t
            last_t = t
            t = self.__next(t,ds,t+dt)
            dt = t - last_t
        # I'm not sure if you want this behavior, but this will
        # cause the spline to always report it's endpoint as a planning point
        yield 1

    def draw(self,plt):
        points = []
        for t in range(1000):
            points.append(self.eval(t/1000.0))
        points.append(self.eval(1))
        x,y = zip(*points)
        plt.plot(x,y)

# A waypoint is simply a position, velocity, and acceleration
class Waypoint:
    def __init__(self,p,v,a):
        self.p = array(p)
        self.v = array(v)
        self.a = array(a)

    def __str__(self):
        pos = "Position: " + str(self.p) + "\n"
        vel = "Velocity: " + str(self.v) + "\n"
        acc = "Acceleration: " + str(self.a)
        return "Waypoint:\n" + pos + vel + acc

# Two waypoints uniquely determine a spline, this constructs that spline
def from_waypoints(initial, final):
    p0 = initial.p
    p5 = final.p
    p1 = 1/5. * initial.v + p0
    p2 = 1/20. * initial.a + 2*p1 - p0
    p4 = p5 - 1/5. * final.v
    p3 = 1/20. * final.a + 2 * p4 - p5
    return Spline(p0,p1,p2,p3,p4,p5)
