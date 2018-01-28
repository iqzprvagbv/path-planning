# Defines a velocity profile, which is the big object we've been
# working towards.
from math import sqrt, ceil, floor
from util import diff
from numpy.linalg import norm

import json

class PlanningPoint:
    def __init__(self,p,t,R,D,h,T=None,V=None,v=None,vr=None,vl=None):
        self.internal_time   = t
        self.R               = R
        self.distance        = D
        self.heading         = h
        self.external_time   = T
        self.max_velocity    = V
        self.actual_velocity = v
        self.right_velocity  = vr
        self.left_velocity   = vl
        self.position        = p
        self.total_time      = None

    def __str__(self):
        return ("Planning Point: " + "\n" +
               "Time: " + str(self.internal_time) + "\n" +
               "Max Velocity: " + str(self.max_velocity) + "\n" +
               "Velocity: " + str(self.actual_velocity) + "\n")

    def compute_max_velocity(self,robot):
        if self.R == 0:
            v = robot.max_velocity
        elif self.R > 0:
            v = (self.R*robot.max_velocity)/(self.R + (robot.width/2.))
        else:
            v = (self.R*robot.max_velocity)/(self.R - (robot.width/2.))

        self.max_velocity = v

    def compute_wheel_velocity(self,robot):
        if self.actual_velocity is None:
            velocity = self.max_velocity
        else:
            velocity = self.actual_velocity

        if self.R == 0:
            self.right_velocity = velocity
            self.left_velocity = velocity
        else:
            self.right_velocity = velocity/self.R*(self.R+robot.width/2)
            self.left_velocity  = velocity/self.R*(self.R-robot.width/2)

    def json_object(self):
        return {"time": self.external_time, "heading": self.heading, "left velcoity": self.left_velocity, "right velocity": self.right_velocity}

class VelocityProfile:
    def __init__(self,path,robot,ds):
        self.path = path
        self.robot = robot
        self.ds = ds
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
        steps = ceil(self.path.l/self.ds)
        step = 1
        progress = 0
        for t in self.path.planning_times(self.ds):
            print '\r[{0}{1}] {2}%'.format('#'*int(progress * 30), '-'*(int((1-progress) * 30)),int(progress*100)),
            R = self.path.curvature_radius(t)
            D = self.path.length(last_t,t)
            point = self.path.eval(t)
            h = self.path.heading(t)
            p = PlanningPoint(point,t,R,D,h)
            p.compute_max_velocity(self.robot)
            p.compute_wheel_velocity(self.robot)
            self.points.append(p)
            last_t = t
            step += 1
            progress = step/steps
        print "Done!"

    def __forward_consistency(self,initial_velocity):
        print "Establishing Forward Consistency..."
        last_velocity = None
        for p in self.points:
            #print "Before: " + str(p.actual_velocity)
            if last_velocity is None:
                #print "Setting Initial Velocity"
                #print "Set Initial Velocity: " + str(initial_velocity)
                #print "Maximal Velocity at Point: " + str(p.max_velocity)
                p.actual_velocity = min(initial_velocity,p.max_velocity)
            else:
                obtainable = sqrt(last_velocity**2+2*self.robot.max_acceleration*p.distance)
                p.actual_velocity = min(p.max_velocity,obtainable)
            #print "After: " + str(p.actual_velocity)
            last_velocity = p.actual_velocity
        print "Done!"

    def __reverse_consistency(self,final_velocity):
        print "Establishing Reverse Consistency..."
        last_velocity = None
        last_distance = None
        for p in reversed(self.points):
            if last_velocity is None:
                p.actual_velocity = min(final_velocity,p.actual_velocity)
            else:
                obtainable = sqrt(last_velocity**2+2*self.robot.max_acceleration*last_distance)
                p.actual_velocity = min(p.actual_velocity,obtainable)

            last_distance = p.distance
            last_velocity = p.actual_velocity
        print "Done!"

    def __establish_timestamps(self):
        print "Establishing Timestamps..."
        last_time = None
        last_velocity = None
        for p in self.points:
            if last_time is None:
                p.external_time = 0
            else:
                p.external_time = last_time + (2*p.distance)/(p.actual_velocity + last_velocity)

            last_time = p.external_time
            last_velocity = p.actual_velocity
        self.total_time = last_time
        print "Done!"

    def __init_wheels(self):
        print "Computing Wheel Velocities..."
        for p in self.points:
            p.compute_wheel_velocity(self.robot)
        print "Done!"

    def __get_max_accel(self):
        lp = None
        max_accel = 0
        for p in self.points:
            if lp is None:
                lp = p
            else:
                dt = p.external_time - lp.external_time
                al = (abs(p.left_velocity - lp.left_velocity))/dt
                ar = (abs(p.right_velocity - lp.right_velocity))/dt
                max_accel = max(max_accel, al, ar)
        return max_accel

    def json(self):
        return json.dumps(self,cls=ProfileEncoder)

class ProfileEncoder(json.JSONEncoder):
    def default(self,obj):
        if isinstance(obj, VelocityProfile):
            output = []
            for p in obj.points:
                output.append(p.json_object())
            return output
        # This will throw an error if it's given the wrong type
        return json.JSONEncoder.default(self, obj)
