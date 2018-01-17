# Defines a velocity profile, which is the big object we've been
# working towards.
from math import sqrt

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns

class PlanningPoint:
    def __init__(self,p,t,R,D,T=None,V=None,v=None,vr=None,vl=None):
        self.internal_time   = t
        self.R               = R
        self.distance        = D
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
            v = (self.R*robot.max_velocity)/(self.R + (robot.width/2))
        else:
            v = (self.R*robot.max_velocity)/(self.R - (robot.width/2))

        self.max_velocity = v

    def compute_wheel_velocity(self,robot):
        if self.R == 0:
            self.right_velocity = self.actual_velocity
            self.left_velocity = self.actual_velocity
        else:
            self.right_velocity = self.actual_velocity/self.R*(self.R+robot.width/2)
            self.left_velocity  = self.actual_velocity/self.R*(self.R-robot.width/2)

class VelocityProfile:
    def __init__(self,path,robot,ds):
        self.path = path
        self.robot = robot
        self.ds = ds
        self.points = []
        self.__init_points()
        self.__forward_consistency(0)
        self.__reverse_consistency(0)
        self.__establish_timestamps()
        self.__init_wheels()

    def __init_points(self):
        print "Initializing Planning Points..."
        last_t = 0
        for t in self.path.planning_times(self.ds):
            R = self.path.curvature_radius(t)
            D = self.path.length(last_t,t)
            point = self.path.eval(t)
            p = PlanningPoint(point,t,R,D)
            p.compute_max_velocity(self.robot)
            self.points.append(p)
            last_t = t
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

    def __draw_curve(self,canvas,planning=False):
        self.path.draw(canvas,segmented=True)
        if planning:
            points = []
            for p in self.points:
                points.append(p.position)
                x,y = zip(*points)
                canvas.plot(x,y,'r.')
        canvas.axis('equal')

    def __draw_velocities(self,canvas):
        t = []
        v = []
        for p in self.points:
            t.append(p.external_time)
            v.append(p.actual_velocity)

        canvas.set_title('Bot Velocity')
        canvas.set_xlabel('Seconds')
        canvas.set_ylabel('Feet Per Second')
        canvas.set_xlim(left=0,right=self.total_time)
        canvas.plot(t,v)

    def __draw_wheels(self, plt):
        t  = []
        vl = []
        vr = []
        for p in self.points:
            t.append(p.external_time)
            vl.append(p.left_velocity)
            vr.append(p.right_velocity)

        plt.set_title('Wheel Velocities')
        plt.set_xlabel('seconds')
        plt.set_ylabel('feet per second')
        plt.set_xlim(left=0,right=self.total_time)
        plt.plot(t,vl,label="Left Wheel")
        plt.plot(t,vr,label="Right Wheel")

    def draw(self):
        sns.set()
        fig = plt.figure()
        gridspec.GridSpec(3,5)

        ax1 = plt.subplot2grid((4,6), (0,0), colspan=3, rowspan=4)
        ax2 = plt.subplot2grid((4,6), (0,3), colspan=3, rowspan=2)
        ax3 = plt.subplot2grid((4,6), (2,3), colspan=3, rowspan=2)

        self.__draw_curve(ax1)
        self.__draw_velocities(ax2)
        self.__draw_wheels(ax3)

        fig.tight_layout()

        plt.legend()

        plt.show()
