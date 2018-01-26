# Creates visualizations

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns

def draw_spline(spline,canvas):
    points = []
    for t in range(1000):
        points.append(spline.eval(t/1000.0))

    x,y = zip(*points)
    canvas.plot(x,y)

def draw_path(path,canvas,segmented=False):
    if segmented:
        for s in path.splines:
            draw_spline(s,canvas)
    else:
        points = []
        for t in range(1000):
            points.append(path.eval(t/1000.0))

        x,y = zip(*points)
        canvas.plot(x,y)

def draw_velocity_profile(vp):
    sns.set()
    fig = plt.figure()
    gridspec.GridSpec(3,5)

    ax1 = plt.subplot2grid((4,6), (0,0), colspan=3, rowspan=4)
    ax2 = plt.subplot2grid((4,6), (0,3), colspan=3, rowspan=2)
    ax3 = plt.subplot2grid((4,6), (2,3), colspan=3, rowspan=2)

    __draw_curve(vp,ax1)
    __draw_wheel_paths(vp,ax1)
    __draw_velocities(vp,ax2)
    __draw_wheel_velocities(vp,ax3)

    fig.tight_layout()

    plt.legend()

    plt.show(block=False)

def __draw_curve(vp,canvas,planning=False,segmented=False):
    draw_path(vp.path,canvas,segmented)
    if planning:
        points = []
        for p in vp.points:
            points.append(p.position)
        x,y = zip(*points)
        canvas.plot(x,y,'r.')
    canvas.axis('equal')

def __draw_velocities(vp,canvas):
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

def __draw_wheel_velocities(vp, canvas):
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

def __draw_wheel_paths(vp,canvas):
    right_points = []
    left_points = []
    for t in range(100):
        pos = vp.path.eval(t/100.)
        normal = vp.path.unit_normal(t/100.)
        left = pos + normal * vp.robot.width/2.
        right = pos - normal * vp.robot.width/2.
        right_points.append(right)
        left_points.append(left)
    lx,ly = zip(*left_points)
    rx,ry = zip(*right_points)
    canvas.plot(lx,ly,'r--')
    canvas.plot(rx,ry,'r--')
