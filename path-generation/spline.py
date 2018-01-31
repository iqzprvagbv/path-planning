"""Contains the implementation of quintic bezier splines"""
from math            import acos
from numpy           import dot, array, clip
from numpy.linalg    import norm
from scipy.integrate import quad
from scipy.optimize  import brentq

class Spline(object):
    """Quintic bezier spline implementation

    Attributes:
        coeffecients : The vector of coeffecients for the polynomial
        total_length : The length of the entire spline

    """
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

    # The parameter functions when dotted with our coeffecient matrix
    # will produce the 0th, 1st, and 2nd derivative respectively
    @staticmethod
    def __parameter(time):
        """Parameter vector for the polynomial

        The parameter vector is a 5x1 vector simply defined as
        [t^0,t^1,...,t^5] it is such that evaluating this vector at time t
        and dotting it with our coeffecient vector evaluates the polynomial at
        time t

        Args:
            time (float) : A number between 0 and 1

        Returns:
            numpy array : An array representing our parameter vector at time t
        """
        return [1, time, time**2, time**3, time**4, time**5]

    @staticmethod
    def __dparameter(time):
        """Parameter vector for the derivative of the polynomial

        The parameter vector is a 5x1 vector simply defined as
        [0,t^0,...,5t^4] it is the derivative of our regular parameter vector.
        This makes it so that evaluating this vector at time t and dotting it
        with our coeffecient vector evaluates the derivative of the polynomial
        at time t

        Args:
            time (float) : A number between 0 and 1

        Returns:
            numpy array : An array representing our parameter vector at time t
        """
        return [0, 1, 2*time, 3*time**2, 4*time**3, 5*time**4]

    @staticmethod
    def __ddparameter(time):
        """Parameter vector for the second derivative of the polynomial

        The parameter vector is a 5x1 vector simply defined as
        [0,0,2t^0,6t^1,...,20t^3] it is the second derivative of our regular
        parameter vector. This makes it so that evaluating this vector at time
        t and dotting it  with our coeffecient vector evaluates the second
        derivative of the polynomial at time t

        Args:
            time (float) : A number between 0 and 1

        Returns:
            numpy array : An array representing our parameter vector at time t
        """
        return [0, 0, 2, 6*time, 12*time**2, 20*time**3]

    def eval(self, time):
        """Evaluates the curve at point in time

        Computes the value of the polynomial at time t by dotting the
        coeffecient matrix with the parameter polynomial. This may not be the
        fastest way to do it as de Casteljau's algorithm exists. May try
        swapping it out once more features are fleshed out, as it stands
        execution time is a non-issue.

        Args:
            time (float) : Time value between 0 and 1
        Returns:
            numpy array : The result of evaluation
        """
        return dot(self.__parameter(time), self.coeffecients)

    def __derivative(self, time):
        """Evaluates the derivative at point in time

        Computes the derivative of the polynomial at time t by dotting the
        coeffecient matrix with the parameter polynomial. This may not be the
        fastest way to do it as the derivative simplifies to the difference of
        two bezier splines and de Casteljau's algorithm exists. Might try to
        optimize later.

        Args:
            time (float) : Time value between 0 and 1
        Returns:
            numpy array : The result of evaluating the derivative
        """
        return dot(self.__dparameter(time), self.coeffecients)

    def __double_derivative(self, time):
        """Evaluates the second derivative at point in time

        Computes the second derivative of the polynomial at time t by dotting
        the coeffecient matrix with the parameter polynomial. This may not be
        the fastest way to do it as the derivative simplifies to the difference
        of two bezier splines and de Casteljau's algorithm exists. Might try to
        optimize later.

        Args:
            time (float) : Time value between 0 and 1
        Returns:
            numpy array : The result of evaluating the second derivative
        """
        return dot(self.__ddparameter(time), self.coeffecients)

    def __curvature(self, time):
        """Computes the curvature at a point in time

        The math here should probably be typed set nice and pretty at some
        point.

        Args:
            time (float) : A time between 0 and 1
        Returns:
            float : The signed curvature at the point time
        """
        term_1 = self.__derivative(time)[0] * self.__double_derivative(time)[1]
        term_2 = self.__derivative(time)[1] * self.__double_derivative(time)[0]
        term_3 = (self.__derivative(time)[0] ** 2) + \
                 (self.__derivative(time)[1] ** 2)
        return (term_1 - term_2)/(term_3 ** 1.5)

    def tangent(self, time):
        """Gives the tangent vector at a point in time

        Not really sure why I wrote it this way, this is essentially just a
        wrapper around the function :func:`~spline.Spline.__derivative`

        Args:
            time (float) : Time between 0 and 1
        Returns:
            numpy array : Tangent vector at time t
        """
        return self.__derivative(time)

    def unit_tangent(self, time):
        """Gives the unit tangent vector at point in Time

        Same as calling :func:`~spline.Spline.__derivative` then normalizing

        Args:
            time (float) : Time between 0 and 1
        Returns:
            numpy array : Tangent vector at point in time
        """
        return self.tangent(time)/norm(self.tangent(time))

    def heading(self, time):
        """Returns the heading of the robot

        Given a point in time return the heading of the robot. The heading is
        measured in radians and is given counter clockwise off of the x-axis,
        i.e. a tangent vector of (1,0) corresponds to a heading of 0 while
        (0,1) is a heading of pi/4.

        Args:
            time (float) : Time between 0 and 1
        Returns:
            float : Heading in radians
        """
        x = dot(self.unit_tangent(time), [1, 0])
        return acos(clip(x, -1, 1))

    def unit_normal(self, time):
        """Calculates unit normal at point in time

        Given a point in time computes the unit normal vector at that point. We
        technically cheat a little bit here, we take the unit tangent normal
        then rotate 90 degrees. Seems to work though.

        Args:
            time (float) : Time between 0 and 1
        Returns:
            numpy array : Unit normal at time
        """

        tangent = self.unit_tangent(time)
        return dot(tangent, [[0, 1], [-1, 0]])

    def curvature_radius(self, time):
        """Returns signed radius of curvature

        Computes the reciprocal to the curvature, choosing to return +inf if
        curvature is 0.

        Args:
            time (float) : Time between 0 and 1
        Returns:
            float : Signed radius of curvature
        """
        try:
            radius = 1/self.__curvature(time)
        except ValueError:
            radius = float('Inf')
        return radius

    def length(self, start, end):
        """Computes the length of a segment of the spline

        Using gaussian quadrature numerically computes the length of the spline
        between start and end.

        Args:
            start (float) : Starting time between 0 and 1
            end (float) : Ending time between start and 1
        Returns:
            float : length of the splite between points start and end
        """
        return quad(lambda t: norm(self.tangent(t)), start, end)[0]

    def __next(self, time, distance):
        """Finds new time such that the distance away from given time is fixed.

        Args:
            float (float) : Time between 0 and 1
            distance (float) : Distance we want to travel from time t
        Returns:
            float : Time t such that distance from time to t is distance
        """
        if self.length(0, time) + distance > self.total_length:
            root = 1
        else:
            fun = lambda x: self.length(time, x) - distance
            root = brentq(fun, time, 1)

        return root

    def planning_times(self, distance):
        """Generates a list of planning times fixed distance apart

        Generates a list of times from 0 to 1 such that the distance from
        s(t_i) to s(t_{i+1}) along the spline is equal to distance.

        Args:
            distance (float) : The distance between each planning point

        Yields:
            float : A time such that the distance between the previous time and
            this time along the spline is the given distance.

        """
        time = 0
        while time < 1:
            yield time
            time = self.__next(time, distance)
        # I'm not sure if you want this behavior, but this will
        # cause the spline to always report it's endpoint as a planning point
        yield 1

# A waypoint is simply a position, velocity, and acceleration
class Waypoint(object):
    """A waypoint for defining splines

    A waypoint for nicer spline generation. Instead of generating splines by
    explicitly defining their control points we can opt to join up two waypoint
    which are a position, velocity, and acceleration vector. The quintic nature
    of our spline will give us a singular unique spline going through the two
    waypoints such that those constraints are met.

    Attributes:
        position (numpy array) : The position of the waypoint
        velocity (numpy array) : Velocity vector at the point
        acceleration (numpy array) : Acceleration vector at the point

    """
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
    """Given two waypoints generate a spline

    Given two waypoints there is a unique quintic bezier spline which goes
    through the waypoints such that it's velocity/acceleration vectors line up
    I should come back and add the equation at somepoint.

    Args:
        initial (Waypoint) : Waypoint for the start of the spline
        final (Waypoint) : Waypoint for the end of the spline
    Returns:
        Spline : A quintic spline through the waypoints

    """
    p_0 = initial.position
    p_5 = final.position
    p_1 = 1/5. * initial.velocity + p_0
    p_2 = 1/20. * initial.acceleration + 2*p_1 - p_0
    p_4 = p_5 - 1/5. * final.velocity
    p_3 = 1/20. * final.acceleration + 2 * p_4 - p_5
    return Spline(p_0, p_1, p_2, p_3, p_4, p_5)
