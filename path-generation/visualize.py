# Creates visualizations

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns

def plot_pairs(canvas, points, fstring=""):
    x,y = zip(*points)
    if fstring == "":
        return canvas.plot(x,y)
    else:
        return canvas.plot(x,y,fstring)

def update_data(line,points):
    x,y = zip(*points)
    line.set_xdata(x)
    line.set_ydata(y)

class Visualizer:

    def __init__(self,path,offset=0):
        sns.set()
        self.curve_data = []
        plt.show()
        fig1 = plt.figure(1)
        gridspec.GridSpec(3,3)
        self.ax1 = plt.subplot2grid((3,3), (0,0), colspan=3, rowspan=3)
        self.ax1.set_xlim(-0.5,20.5)
        self.ax1.set_ylim(-0.5,20.5)
        #self.ax1.axis('equal')
        what = self.draw_path(path,self.ax1,False,offset)
        self.path_lines={"center":what[0],"left":what[1],"right":what[2]}
        plt.show(block=False)
        #self.__draw_curve(vp,ax1)

    def update_path(self,path,offset=0):
        points = []
        for t in range(1000):
            points.append(path.eval(t/1000.0))

        if offset:
            lpoints = []
            rpoints = []
            for t in range(1000):
                pos = path.eval(t/1000.)
                normal = path.unit_normal(t/1000.)
                lpoints.append(pos+normal*offset)
                rpoints.append(pos-normal*offset)
            update_data(self.path_lines["left"],lpoints)
            update_data(self.path_lines["right"],rpoints)

        update_data(self.path_lines["center"],points)
        plt.draw()


    def draw_spline(self,spline,canvas):
        points = []
        for t in range(1000):
            points.append(spline.eval(t/1000.0))

        plot_pairs(points,canvas)

    def draw_path(self,path,canvas,segmented=False,offset=0):
        if segmented:
            for s in path.splines:
                self.draw_spline(s,canvas)
        else:
            points = []
            for t in range(1000):
                points.append(path.eval(t/1000.0))

            if offset:
                lpoints = []
                rpoints = []
                for t in range(1000):
                    pos = path.eval(t/1000.)
                    normal = path.unit_normal(t/1000.)
                    lpoints.append(pos+normal*offset)
                    rpoints.append(pos-normal*offset)
                left = plot_pairs(canvas,lpoints,'r--')
                right = plot_pairs(canvas,rpoints,'r--')

            center = plot_pairs(canvas,points)
            return center + left + right

    def draw_velocity_profile(self,vp):

        #self.__draw_wheel_paths(vp,ax1)

        fig2 = plt.figure(2)
        ax2 = plt.subplot2grid((2,2), (0,0), colspan=2, rowspan=1)
        ax3 = plt.subplot2grid((2,2), (1,0), colspan=2, rowspan=1)

        self.__draw_velocities(vp,ax2)
        self.__draw_wheel_velocities(vp,ax3)

        fig2.tight_layout()

        plt.legend()

        plt.show(block=False)

    def __draw_curve(self,vp,canvas,planning=False,segmented=False):
        self.draw_path(vp.path,canvas,segmented,vp.robot.width/2.)
        if planning:
            points = []
            for p in vp.points:
                points.append(p.position)
            plot_pairs(canvas,points,'r.')
        canvas.axis('equal')

    def __draw_velocities(self,vp,canvas):
        t = []
        v = []
        for p in vp.points:
            t.append(p.external_time)
            v.append(p.actual_velocity)

        canvas.set_title('Bot Velocity')
        canvas.set_xlabel('Seconds')
        canvas.set_ylabel('Feet Per Second')
        canvas.set_xlim(left=0,right=vp.total_time)
        canvas.plot(t,v)

    def __draw_wheel_velocities(self,vp, canvas):
        t  = []
        vl = []
        vr = []
        for p in vp.points:
            t.append(p.external_time)
            vl.append(p.left_velocity)
            vr.append(p.right_velocity)

        canvas.set_title('Wheel Velocities')
        canvas.set_xlabel('seconds')
        canvas.set_ylabel('feet per second')
        canvas.set_xlim(left=0,right=vp.total_time)
        canvas.plot(t,vl,label="Left Wheel")
        canvas.plot(t,vr,label="Right Wheel")

    def __draw_wheel_paths(self,vp,canvas):
        right_points = []
        left_points = []
        for t in range(100):
            pos = vp.path.eval(t/100.)
            normal = vp.path.unit_normal(t/100.)
            left = pos + normal * vp.robot.width/2.
            right = pos - normal * vp.robot.width/2.
            right_points.append(right)
            left_points.append(left)

        plot_pairs(canvas,left_points,'r--')
        plot_pairs(canvas,right_points,'r--')
