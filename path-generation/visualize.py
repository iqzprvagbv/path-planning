# Creates visualizations

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns

def plot_pairs(canvas, points, fstring=""):
    if points:
        x, y = zip(*points)
    else:
        x = []
        y = []

    if fstring == "":
        return canvas.plot(x, y)

    return canvas.plot(x, y, fstring)

def update_data(line, points):
    if points:
        x, y = zip(*points)
    else:
        x = []
        y = []

    line.set_xdata(x)
    line.set_ydata(y)

class Visualizer(object):

    def __init__(self, path, offset=0):
        sns.set()
        self.curve_data = []
        plt.show()
        self.fig1 = plt.figure(1)
        gridspec.GridSpec(3, 3)
        self.ax1 = plt.subplot2grid((3, 3), (0, 0), colspan=3, rowspan=3)
        self.ax1.set_xlim(-0.5, 20.5)
        self.ax1.set_ylim(-0.5, 20.5)
        what = self.draw_path(path, self.ax1, offset)
        self.path_lines = {"center":what[0], "left":what[1], "right":what[2]}
        #plt.show(block=False)
        #self.__draw_curve(vp,ax1)

    @staticmethod
    def show():
        plt.show(block=False)

    def update_path(self, path, offset=0):
        if path is None:
            update_data(self.path_lines["left"], [])
            update_data(self.path_lines["right"], [])
            update_data(self.path_lines["center"], [])

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
                update_data(self.path_lines["left"], lpoints)
                update_data(self.path_lines["right"], rpoints)

            update_data(self.path_lines["center"], points)

        plt.draw()

    @staticmethod
    def draw_spline(spline, canvas):
        points = []
        for t in range(1000):
            points.append(spline.eval(t/1000.0))

        plot_pairs(points, canvas)

    @staticmethod
    def draw_path(path, canvas, offset=0):
        if path is None:
            center = plot_pairs(canvas, [])
            left = plot_pairs(canvas, [], 'r--')
            right = plot_pairs(canvas, [], 'r--')
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
                left = plot_pairs(canvas, lpoints, 'r--')
                right = plot_pairs(canvas, rpoints, 'r--')

            center = plot_pairs(canvas, points)

        return center + left + right

    def draw_velocity_profile(self, velocity_profile):
        fig2 = plt.figure(2)
        ax2 = plt.subplot2grid((2, 2), (0, 0), colspan=2, rowspan=1)
        ax3 = plt.subplot2grid((2, 2), (1, 0), colspan=2, rowspan=1)

        self.__draw_velocities(velocity_profile, ax2)
        self.__draw_wheel_velocities(velocity_profile, ax3)

        fig2.tight_layout()

        plt.legend()

        plt.show(block=False)

    def __draw_curve(self, profile, canvas, planning=False):
        self.draw_path(profile.path, canvas, profile.robot.width/2.)
        if planning:
            points = []
            for point in profile.points:
                points.append(point.position)
            plot_pairs(canvas, points, 'r.')
        canvas.axis('equal')

    @staticmethod
    def __draw_velocities(profile, canvas):
        times = []
        velocities = []
        for point in profile.points:
            times.append(point.external_time)
            velocities.append(point.actual_velocity)

        canvas.set_title('Bot Velocity')
        canvas.set_xlabel('Seconds')
        canvas.set_ylabel('Feet Per Second')
        canvas.set_xlim(left=0, right=profile.total_time)
        canvas.plot(times, velocities)

    @staticmethod
    def __draw_wheel_velocities(profile, canvas):
        times = []
        left_velocities = []
        right_velocities = []
        for point in profile.points:
            times.append(point.external_time)
            left_velocities.append(point.left_velocity)
            right_velocities.append(point.right_velocity)

        canvas.set_title('Wheel Velocities')
        canvas.set_xlabel('seconds')
        canvas.set_ylabel('feet per second')
        canvas.set_xlim(left=0, right=profile.total_time)
        canvas.plot(times, left_velocities, label="Left Wheel")
        canvas.plot(times, right_velocities, label="Right Wheel")

    @staticmethod
    def __draw_wheel_paths(profile, canvas):
        right_points = []
        left_points = []
        for t in range(100):
            pos = profile.path.eval(t/100.)
            normal = profile.path.unit_normal(t/100.)
            left = pos + normal * profile.robot.width/2.
            right = pos - normal * profile.robot.width/2.
            right_points.append(right)
            left_points.append(left)

        plot_pairs(canvas, left_points, 'r--')
        plot_pairs(canvas, right_points, 'r--')
