# This file defines the underlying spline we use to represent the paths
# They are quintic bezier curves

from math            import acos
from numpy           import dot, array, clip
from numpy.linalg    import norm
from scipy.integrate import quad
from scipy.optimize  import brentq

class Spline(object):
    def __init__(self, p_0, p_1, p_2, p_3, p_4, p_5):
        # C is the coeffecient matrix for our spline
        # The polynomial looks like:
        # c[0] + c[1]t + c[2]t^2 + c[3]t^3 + ...
        self.coeffecients = []
        self.coeffecients.append(p_0)
        self.coeffecients.append(-5 * p_0 +  5 * p_1)
        self.coeffecients.append(10 * p_0 - 20 * p_1 + 10 * p_2)
        self.coeffecients.append(-10 * p_0 + 30 * p_1 - 30 * p_2 + 10 * p_3)
        self.coeffecients.append(5 * p_0 - 20 * p_1 + 30 * p_2 - 20 * p_3 + \
                                 5 * p_4)
        self.coeffecients.append(-1 * p_0 +  5 * p_1 - 10 * p_2 + 10 * p_3 - \
                                 5  * p_4 + p_5)

        self.total_length = self.length(0, 1)

        self.__last_dt = 0 #This feels messy... only one function really uses it

    # The parameter functions when dotted with our coeffecient matrix
    # will produce the 0th, 1st, and 2nd derivative respectively
    @staticmethod
    def __parameter(time):
        return [1, time, time**2, time**3, time**4, time**5]

    @staticmethod
    def __dparameter(time):
        return [0, 1, 2*time, 3*time**2, 4*time**3, 5*time**4]

    @staticmethod
    def __ddparameter(time):
        return [0, 0, 2, 6*time, 12*time**2, 20*time**3]

    def eval(self, time):
        return dot(self.__parameter(time), self.coeffecients)

    def __derivative(self, time):
        return dot(self.__dparameter(time), self.coeffecients)

    def __double_derivative(self, time):
        return dot(self.__ddparameter(time), self.coeffecients)

    def __curvature(self, time):
        term_1 = self.__derivative(time)[0] * self.__double_derivative(time)[1]
        term_2 = self.__derivative(time)[1] * self.__double_derivative(time)[0]
        term_3 = (self.__derivative(time)[0] ** 2) + \
                 (self.__derivative(time)[1] ** 2)
        return (term_1 - term_2)/(term_3 ** 1.5)

    def tangent(self, time):
        return self.__derivative(time)

    def unit_tangent(self, time):
        return self.tangent(time)/norm(self.tangent(time))

    def heading(self, time):
        x = dot(self.unit_tangent(time), [1, 0])
        return acos(clip(x, -1, 1))

    def unit_normal(self, time):
        tangent = self.unit_tangent(time)
        return dot(tangent, [[0, 1], [-1, 0]])

    def curvature_radius(self, time):
        return 1/self.__curvature(time)

    def length(self, start, end):
        return quad(lambda t: norm(self.tangent(t)), start, end)[0]

    # Given a time t and a distance ds this function approximates
    # an s such that arc length from t to s is ds. Guess is a
    # provided approximation of s for the algorithm to start with
    def __next(self, time, distance):
        if self.length(0, time) + distance > self.total_length:
            root = 1
        else:
            f = lambda x: self.length(time, x) - distance
            root = brentq(f, time, 1)

        return root

    # Generates a list of t such that the archlength between consecutive
    # them are equal.
    def planning_times(self, distance):
        t = 0
        while t < 1:
            yield t
            t = self.__next(t, distance)
        # I'm not sure if you want this behavior, but this will
        # cause the spline to always report it's endpoint as a planning point
        yield 1

# A waypoint is simply a position, velocity, and acceleration
class Waypoint(object):
    def __init__(self, position, velocity, acceleration):
        self.position = array(position)
        self.velocity = array(velocity)
        self.acceleration = array(acceleration)

    def __str__(self):
        pos = "Position: " + str(self.position) + "\n"
        vel = "Velocity: " + str(self.velocity) + "\n"
        acc = "Acceleration: " + str(self.acceleration)
        return "Waypoint:\n" + pos + vel + acc

# Two waypoints uniquely determine a spline, this constructs that spline
def from_waypoints(initial, final):
    p_0 = initial.position
    p_5 = final.position
    p_1 = 1/5. * initial.velocity + p_0
    p_2 = 1/20. * initial.acceleration + 2*p_1 - p_0
    p_4 = p_5 - 1/5. * final.velocity
    p_3 = 1/20. * final.acceleration + 2 * p_4 - p_5
    return Spline(p_0, p_1, p_2, p_3, p_4, p_5)
