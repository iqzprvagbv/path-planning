from path      import from_waypoints
from profile   import VelocityProfile
from robot     import Robot
from spline    import Waypoint
from visualize import Visualizer


import numpy as np
import util
import cmd

# Width of the robot in feet
robot_width = 2
# Max Velocity of robot in fps
max_velocity = 15
# Max acceleration of robot in fps^2
max_acceleration = 15

robot = Robot(robot_width,max_velocity,max_acceleration)

# Way points for the curve to hit
# Format: Waypoint(position, velocity vector, acceleration vector)
waypoints = []
waypoints.append(Waypoint((1,0),(0,10),(1,0)))
waypoints.append(Waypoint((6,5),(0,10),(-1,0)))
#waypoints.append(Waypoint((0,10),(0,10),(1,0)))
#waypoints.append(Waypoint((15,10),(0,-10),(-1,0)))
#waypoints.append(Waypoint((10,5),(0,-10),(1,0)))
#waypoints.append(Waypoint((15,0),(0,-10),(-1,0)))
#waypoints.append(Waypoint((0,0),(0,10),(1,0)))

path = from_waypoints(waypoints)
visualize = Visualizer(path,offset=robot.width/2.)

intro_message = "Hello, type help to see list of commands"

def update_path():
    visualize.update_path(path,offset=robot.width/2.)

class Prompt(cmd.Cmd):

    def do_waypoint(self, args):
        """ Manipulates the waypoints:\n waypoint : will list waypoints\n waypoint remove n : removes the nth waypoint (0 indexed)\n waypoint add (px,py) (vx,vy) (ay,ax) : adds waypoint with position (px,py), velocity (vx,vy), and acceleration (ax,ay)\n waypoint clear : deletes all waypoints """
        global path
        if args:
            args = args.split(' ')

        else:
            args = None

        if args is None:
            if len(waypoints) == 0:
                print "No waypoints"
            else:
                for wp in waypoints:
                    print wp

        elif len(args) == 1:
            if args[0] == "clear":
                del waypoints[:]
            else:
                print " Couldn't parse, try help waypoint for more info"

        elif len(args) == 2:
            if args[0] == "remove":
                try:
                    index = int(args[1])
                    del waypoints[index]
                    path = from_waypoints(waypoints)
                    update_path
                except ValueError:
                    print " Could not parse", args[1]
            else:
                print " Couldn't parse, try help waypoint for more info"

        elif len(args) == 4:
            if args[0] == "add":
                try:
                    position     = np.fromstring(args[1][1:-1],sep=',')
                    velocity     = np.fromstring(args[2][1:-1],sep=',')
                    acceleration = np.fromstring(args[3][1:-1],sep=',')
                    wp = Waypoint(position, velocity, acceleration)
                    print " Adding waypoint:"
                    print wp
                    waypoints.append(wp)
                    path = from_waypoints(waypoints)
                except ValueError:
                    print " Failed to parse one of the vectors"
            else:
                print " Couldn't parse, try help waypoint for more info"

        else:
            print " Couldn't parse, try help waypoint for more info"

        update_path()

    def do_robot(self,args):
        """ Displays and sets robot data:\n robot : will list all robot attributes\n robot [attribute] : will display value for [attribute]\n robot [attribute] x : sets [attribute] to x \n\n Attributes \n ==========\n width : Distance from left wheel to right wheel in feet\n velocity : Max velocity of robot in feet per second\n acceleration : Max acceleration of robot in feet per second squared """
        if args:
            args = args.split(" ")
        else:
            args = None

        if args is None:
            print robot
        elif len(args) == 1:
            attribute = args[0]
            if attribute == "width":
                print " Width:", robot.width, "feet"
            elif attribute == "velocity":
                print " Velocity:", robot.max_velocity, "feet per second"
            elif attribute == "acceleration":
                print " Acceleration:", robot.max_acceleration, "feet per second squared"
            else:
                print " Could not recognize attribute:", attribute, "\n try help robot for more information on this command"
        elif len(args) == 2:
            attribute = args[0]

            try:
                value = float(args[1])
            except ValueError:
                print " Could not parse:", args[1]
                return

            if attribute == "width":
                robot.width = value
                print " Set width to", value, "feet"
            elif attribute == "velocity":
                robot.velocity = value
                print " Set velocity to", value, "feet per second"
            elif attribute == "acceleration":
                robot.acceleration = value
                print " Set acceleration to", value, "feet per second squared"
            else:
                print " Could not recognize attribute:", attribute, "\n try help robot for more information on this command"
        else:
            print " Could not recogize command try \'help robot\' for more information"
        update_path()

    def do_compute(self,args):
        """ Computes the velocity profile for the path currently defined.\n compute ds : ds is the distance between between planning points in feet"""
        if args:
            try:
                ds = float(args)
                vp = VelocityProfile(path,robot,ds)
                visualize.draw_velocity_profile(vp)
            except ValueError:
                print " Failed to parse", args
        else:
            print " Failed to parse, try \'help compute\' for more help"

    def do_quit(self, line):
        """Quits the program"""
        return True

if __name__ == '__main__':
    prompt = Prompt()
    prompt.prompt = "> "
    prompt.cmdloop(intro_message)
