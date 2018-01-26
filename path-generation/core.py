from path      import from_waypoints
from profile   import VelocityProfile
from robot     import Robot
from spline    import Waypoint
from visualize import draw_velocity_profile

import util
import cmd

# Width of the robot in feet
robot_width = 2
# Max Velocity of robot in fps
max_velocity = 15
# Max acceleration of robot in fps^2
max_acceleration = 15

robot = Robot(robot_width,max_velocity,max_acceleration)

class Prompt(cmd.Cmd):
    def do_waypoint(self, args):
        """ Manipulates the waypoints:\n waypoint : will list waypoints\n waypoint remove n : removes the nth waypoint (0 indexed)\n waypoint add (px,py) (vx,vy) (ay,ax) : adds waypoint with position (px,py), velocity (vx,vy), and acceleration (ax,ay)\n waypoint clear : deletes all waypoints """


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

    def do_quit(self, line):
        """Quits the program"""
        return True

if __name__ == '__main__':
    prompt = Prompt()
    prompt.prompt = "> "
    prompt.cmdloop()


# Way points for the curve to hit
# Format: Waypoint(position, velocity vector, acceleration vector)
waypoints = []
waypoints.append(Waypoint((0,0),(0,10),(1,0)))
waypoints.append(Waypoint((5,5),(0,10),(-1,0)))
waypoints.append(Waypoint((0,10),(0,10),(1,0)))
waypoints.append(Waypoint((15,10),(0,-10),(-1,0)))
waypoints.append(Waypoint((10,5),(0,-10),(1,0)))
waypoints.append(Waypoint((15,0),(0,-10),(-1,0)))
waypoints.append(Waypoint((0,0),(0,10),(1,0)))

# Distance between planning points in feet
ds = 0.1


#path = from_waypoints(waypoints)
#vp = VelocityProfile(path,robot,ds)
#draw_velocity_profile(vp)
