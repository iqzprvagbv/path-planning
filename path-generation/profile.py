# Defines a velocity profile, which is the big object we've been
# working towards.
from math import sqrt, ceil

import json

class PlanningPoint(object):
    # pylint: disable=too-many-instance-attributes
    # planning points unfortunately require this much data

    def __init__(self, position, time, radius, distance, heading):
        # pylint: disable=bad-whitespace
        # this next block is unreadable without the spacing

        self.radius               = radius
        self.heading         = heading
        self.position        = position
        self.distance        = distance
        self.internal_time   = time
        self.total_time      = None
        self.external_time   = None
        self.max_velocity    = None
        self.left_velocity   = None
        self.right_velocity  = None
        self.actual_velocity = None

    def __str__(self):
        return ("Planning Point: " + "\n" +
                "Time: " + str(self.internal_time) + "\n" +
                "Max Velocity: " + str(self.max_velocity) + "\n" +
                "Velocity: " + str(self.actual_velocity) + "\n")

    def compute_max_velocity(self, robot):
        if self.radius == 0:
            velocity = robot.max_velocity
        elif self.radius > 0:
            velocity = (self.radius*robot.max_velocity)/(self.radius + (robot.width/2.))
        else:
            velocity = (self.radius*robot.max_velocity)/(self.radius - (robot.width/2.))

        self.max_velocity = velocity

    def compute_wheel_velocity(self, robot):
        if self.actual_velocity is None:
            velocity = self.max_velocity
        else:
            velocity = self.actual_velocity

        if self.radius == 0:
            self.right_velocity = velocity
            self.left_velocity = velocity
        else:
            self.right_velocity = velocity/self.radius*(self.radius+robot.width/2)
            self.left_velocity = velocity/self.radius*(self.radius-robot.width/2)

    def json_object(self):
        return {"time": self.external_time,
                "heading": self.heading,
                "left velcoity": self.left_velocity,
                "right velocity": self.right_velocity}

class VelocityProfile(object):
    def __init__(self, path, robot, distance):
        self.path = path
        self.robot = robot
        self.distance = distance
        self.points = []
        self.__init_points()
        #broken
        #self.__establish_accel()
        # Dirty Hack
        actual_max_accel = self.robot.max_acceleration
        current_max_accel = float('inf')
        while current_max_accel > actual_max_accel:
            print "Ensuring Consitency of Wheels"
            self.__forward_consistency(0)
            self.__reverse_consistency(0)
            self.__establish_timestamps()
            self.__init_wheels()
            current_max_accel = self.__get_max_accel()
            self.robot.max_acceleration = 3./4 * self.robot.max_acceleration

    def __init_points(self):
        print "Initializing Planning Points..."
        last_t = 0
        steps = ceil(self.path.total_length/self.distance)
        step = 1
        progress = 0
        for t in self.path.planning_times(self.distance):
            print '\r[{0}{1}] {2}%'.format('#'*int(progress * 30),
                                           '-'*(int((1-progress) * 30)),
                                           int(progress*100)),
            radius = self.path.curvature_radius(t)
            distance = self.path.length(last_t, t)
            position = self.path.eval(t)
            heading = self.path.heading(t)
            point = PlanningPoint(position, t, radius, distance, heading)
            point.compute_max_velocity(self.robot)
            point.compute_wheel_velocity(self.robot)
            self.points.append(point)
            last_t = t
            step += 1
            progress = step/steps
        print "Done!"

    def __forward_consistency(self, initial_velocity):
        print "Establishing Forward Consistency..."
        last_velocity = None
        for point in self.points:
            if last_velocity is None:
                point.actual_velocity = min(initial_velocity, point.max_velocity)
            else:
                obtainable = sqrt(last_velocity**2+2*self.robot.max_acceleration*point.distance)
                point.actual_velocity = min(point.max_velocity, obtainable)
            last_velocity = point.actual_velocity
        print "Done!"

    def __reverse_consistency(self, final_velocity):
        print "Establishing Reverse Consistency..."
        last_velocity = None
        last_distance = None
        for point in reversed(self.points):
            if last_velocity is None:
                point.actual_velocity = min(final_velocity, point.actual_velocity)
            else:
                obtainable = sqrt(last_velocity**2+2*self.robot.max_acceleration*last_distance)
                point.actual_velocity = min(point.actual_velocity, obtainable)

            last_distance = point.distance
            last_velocity = point.actual_velocity
        print "Done!"

    def __establish_timestamps(self):
        print "Establishing Timestamps..."
        last_time = None
        last_velocity = None
        for point in self.points:
            if last_time is None:
                point.external_time = 0
            else:
                dt = (2*point.distance)/(point.actual_velocity + last_velocity)
                point.external_time = last_time + dt

            last_time = point.external_time
            last_velocity = point.actual_velocity
        self.total_time = last_time
        print "Done!"

    def __init_wheels(self):
        print "Computing Wheel Velocities..."
        for point in self.points:
            point.compute_wheel_velocity(self.robot)
        print "Done!"

    def __get_max_accel(self):
        last_point = None
        max_accel = 0
        for point in self.points:
            if last_point is None:
                last_point = point
            else:
                dt = point.external_time - last_point.external_time
                left_accel = (abs(point.left_velocity -
                                  last_point.left_velocity))/dt
                right_accel = (abs(point.right_velocity -
                                   last_point.right_velocity))/dt
                max_accel = max(max_accel, left_accel, right_accel)
        return max_accel

class ProfileEncoder(json.JSONEncoder):
    # pylint: disable=arguments-differ
    # pylint: disable=method-hidden
    # This code is copy pasted from the official python docs, I assume it's fine
    def default(self, obj):
        if isinstance(obj, VelocityProfile):
            output = []
            for point in obj.points:
                output.append(point.json_object())
            return output
        # This will throw an error if it's given the wrong type
        return json.JSONEncoder.default(self, obj)
