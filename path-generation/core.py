import json
import cmd

from profile   import VelocityProfile, ProfileEncoder
from path      import from_waypoints
from robot     import Robot
from spline    import Waypoint
from visualize import Visualizer

import numpy as np

INTRO_MESSAGE = "Hello, type help to see list of commands"

class Prompt(cmd.Cmd):

    def __init__(self):
        self.waypoints = []
        self.profile = None
        self.path = None
        self.robot = Robot(2, 15, 10)
        self.visualize = Visualizer(self.path, offset=self.robot.width/2.)
        cmd.Cmd.__init__(self)

    def remove_waypoint(self, index):
        try:
            index = int(index)
            del self.waypoints[index]
            self.path = from_waypoints(self.waypoints)
            self.update_path()
        except ValueError:
            print " Could not parse", index

    def add_waypoint(self, position, velocity, acceleration):
        try:
            position = np.fromstring(position[1:-1], sep=',')
            velocity = np.fromstring(velocity[1:-1], sep=',')
            acceleration = np.fromstring(acceleration[3][1:-1], sep=',')
            waypoint = Waypoint(position, velocity, acceleration)
            print " Adding waypoint:"
            print waypoint
            self.waypoints.append(waypoint)
            self.path = from_waypoints(self.waypoints)
        except ValueError:
            print " Failed to parse one of the vectors"

    def update_robot(self, attribute, value):
        if attribute == "width":
            self.robot.width = value
            print " Set width to", value, "feet"
        elif attribute == "velocity":
            self.robot.velocity = value
            print " Set velocity to", value, "feet per second"
        elif attribute == "acceleration":
            self.robot.acceleration = value
            print " Set acceleration to", value, "feet per second squared"
        else:
            print " Could not recognize attribute:", attribute, "\n try help robot for more information on this command"

    def get_attribute(self, attribute):
        if attribute == "width":
            print " Width:", self.robot.width, "feet"
        elif attribute == "velocity":
            print " Velocity:", self.robot.max_velocity, "feet per second"
        elif attribute == "acceleration":
            print " Acceleration:", self.robot.max_acceleration, "feet per second squared"
        else:
            print " Could not recognize attribute:", attribute, "\n try help robot for more information on this command"

    def update_path(self):
        self.visualize.update_path(self.path, offset=self.robot.width/2.)

    def help_intro(self):
        print "If you some how stumbled upon this program and don't know what it is, details here: https://github.com/iqzprvagbv/path-planning/ \n"
        print "Here's the tl;dr on how to use this program\n"
        print "Set the robot parameters width, maximum velocity, and acceleration using \'robot [parameter] [value]\'\n"
        print "Add waypoints using the commands \'waypoint position velocity accel\' If the command show is run before this you will be able to see the path in realtime. Fixing mistakes can be done with \'waypoint remove n\'\n"
        print "Compute the numerical velocity profile with the command \'compute ds\' where ds is the distance between planning points\n"
        print "If the plots look acceptable save the profile using \'save profile_name\' which will save the file to profile_name.json\n"
        print "The command 'clear' will reset everything but the robot and let you plot again, \'quit\' closes the program, and \'help [command]\' will display help text and options for every command. Enjoy."

    def print_waypoints(self):
        if self.waypoints:
            for waypoint in self.waypoints:
                print waypoint
        else:
            print "No waypoints"

    def do_waypoint(self, args):
        """ Manipulates the waypoints:\n waypoint : will list waypoints\n waypoint remove n : removes the nth waypoint (0 indexed)\n waypoint add (px,py) (vx,vy) (ay,ax) : adds waypoint with position (px,py), velocity (vx,vy), and acceleration (ax,ay)\n waypoint clear : deletes all waypoints """

        if args:
            args = args.split(' ')

        else:
            args = None

        if args is None:
            self.print_waypoints()

        elif args[0] == "clear" and len(args) == 1:
            del self.waypoints[:]

        elif args[0] == "remove" and len(args) == 2:
            self.remove_waypoint(args[1])

        elif args[0] == "add" and len(args) == 4:
            self.add_waypoint(args[1], args[2], args[3])

        else:
            print " Couldn't parse, try help waypoint for more info"

        self.update_path()

    def do_robot(self, args):
        """ Displays and sets robot data:\n robot : will list all robot attributes\n robot [attribute] : will display value for [attribute]\n robot [attribute] x : sets [attribute] to x \n\n Attributes \n ==========\n width : Distance from left wheel to right wheel in feet\n velocity : Max velocity of robot in feet per second\n acceleration : Max acceleration of robot in feet per second squared """
        if args:
            args = args.split(" ")
        else:
            args = None

        if args is None:
            print self.robot
        elif len(args) == 1:
            attribute = args[0]
            self.get_attribute(attribute)
        elif len(args) == 2:
            attribute = args[0]
            try:
                value = float(args[1])
                self.update_robot(attribute, value)
            except ValueError:
                print " Could not parse:", args[1]

        else:
            print " Could not recogize command try \'help robot\' for more information"
        self.update_path()

    def do_compute(self, args):
        """ Computes the velocity profile for the path currently defined.\n compute ds : ds is the distance between between planning points in feet"""
        if args:
            try:
                distance = float(args)
                self.profile = VelocityProfile(self.path, self.robot, distance)
                self.visualize.draw_velocity_profile(self.profile)
            except ValueError:
                print " Failed to parse", args
        else:
            print " Failed to parse, try \'help compute\' for more help"

    def do_show(self, args):
        """ Displays the plots. If the plots close this wont' actually reopen them at the moment. That evidently requires embedding matplotlib graphs in some gui interface and I'm lazy"""
        if args:
            print "Show does not take arguments"
        else:
            self.visualize.show()

    def do_save(self, args):
        """Saves the current velocity profile.\n save [name] : saves to name.json"""
        if args:
            name = args + ".json"
        else:
            name = "profile.jsoin"
        print "saving to", name

        with open(name, 'w') as output:
            json.dump(self.profile, output, cls=ProfileEncoder)

    def do_quit(self, line):
        """Quits the program"""
        if line:
            print "Quit does not take any arguments"
        else:
            return True

if __name__ == '__main__':
    PROMPT = Prompt()
    PROMPT.prompt = "> "
    PROMPT.cmdloop(INTRO_MESSAGE)
