import json
import cmd

import numpy as np

from path               import from_waypoints
from robot              import Robot
from spline             import Waypoint
from visualize          import Visualizer
from velocity_profile   import VelocityProfile, ProfileEncoder


INTRO_MESSAGE = "Hello, type help to see list of commands"

class Prompt(cmd.Cmd):

    def __init__(self):
        self.waypoints = []
        self.profile = None
        self.path = None
        self.robot = Robot(2, 15, 10)
        self.visualize = Visualizer(self.path, offset=self.robot.width/2.)
        cmd.Cmd.__init__(self)

    def update_robot(self, attribute, value):
        """Updates one of the robots attributes

        Args:
            attribute: A string representing which value to modify
            value: A float to be inserted
        """
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
            print " Could not recognize attribute:", attribute
            print " try help robot for more information on this command"

    def print_attribute(self, attribute):
        """Prints one of the robots attributes

        Displays the requested attribute to the console

        Args:
            attribute: A string representing one of the robots properties
        """
        if attribute == "width":
            print " Width:", self.robot.width, "feet"
        elif attribute == "velocity":
            print " Velocity:", self.robot.max_velocity, \
                  "feet per second"
        elif attribute == "acceleration":
            print " Acceleration:", self.robot.max_acceleration, \
                  "feet per second squared"
        else:
            print " Could not recognize attribute:", attribute, \
                  " try help robot for more information on this command"

    def remove_waypoint(self, index):
        """Removes a waypoint

        This is meant to be called by the waypoint remove command. It parses
        the index into an integer (hopefully) and then removes the waypoint.

        Args:
            index: The index of the waypoint to be removed, 0 indexed.
        """

        try:
            index = int(index)
            del self.waypoints[index]
            self.path = from_waypoints(self.waypoints)
            self.update_path()
        except ValueError:
            print " Could not parse", index

    @staticmethod
    def parse_vector(vector):
        """Parses a string into a numpy array

        Chops off the first and last character and then parses into a numpy
        array. This results in potentially weird outcomes.

        Args:
            vector: A string in the form (1,2,...,n), [1,2,...,n],
                <1,2,...,n> etc

        Returns:
            array: A numpy array

        Raises:
            ValueError: If the provided string can not be coerced into
                an array
        """
        try:
            return np.fromstring(vector[1,-1], sep=",")
        except ValueError:
            raise ValueError("Could not parse: " + vector)

    def add_waypoint(self, position, velocity, acceleration):
        """Adds a given waypoint

        This is meant to be called from the waypoint add command. It parses
        the input into vectors then adds the waypoint.

        Args:
            position (str): Represents the position in 2d space of the waypoint.
                Should be parsable by :func:`~core.Prompt.parse_vector`
            velocity (str): Represents the velocity in 2d space of the waypoint.
                Should be parsable by :func:`~core.Prompt.parse_vector`
            acceleration (str): Represents the acceleration in 2d space of the
                waypoint. Should be parsable by
                :func:`~core.Prompt.parse_vector`
        """
        try:
            position = self.parse_vector(position)
            velocity = self.parse_vector(velocity)
            acceleration = self.parse_vector(acceleration)
            waypoint = Waypoint(position, velocity, acceleration)
            print " Adding waypoint:"
            print waypoint
            self.waypoints.append(waypoint)
            self.path = from_waypoints(self.waypoints)
        except ValueError as err:
            print err

    def update_path(self):
        """Updates and redraws the path"""
        self.visualize.update_path(self.path, offset=self.robot.width/2.)

    def print_waypoints(self):
        """Prints all of the waypoints."""
        if self.waypoints:
            for waypoint in self.waypoints:
                print waypoint
        else:
            print "No waypoints"

    @staticmethod
    def help_waypoint():
        """Displays the help text for the waypoint command."""
        print " Manipulates the waypoints:"
        print " waypoint : will list waypoints"
        print " waypoint remove n : removes the nth waypoint (0 indexed)"
        print " waypoint add (px,py) (vx,vy) (ay,ax) :",\
              " adds waypoint with position (px,py), velocity (vx,vy),",\
              " and acceleration (ax,ay)"
        print " waypoint clear : deletes all waypoints "

    def do_waypoint(self, args):
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

    @staticmethod
    def help_robot():
        print " Displays and sets robot data:"
        print " robot : will list all robot attributes"
        print " robot [attribute] : will display value for [attribute]"
        print " robot [attribute] x : sets [attribute] to x \n"
        print " Attributes"
        print " =========="
        print " width : Distance from left wheel to right wheel in feet"
        print " velocity : Max velocity of robot in feet per second"
        print " acceleration : Max acceleration of robot",\
              " in feet per second squared "

    def do_robot(self, args):
        if args:
            args = args.split(" ")
        else:
            args = None

        if args is None:
            print self.robot
        elif len(args) == 1:
            attribute = args[0]
            self.print_attribute(attribute)
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

    @staticmethod
    def help_compute():
        print " Computes the velocity profile of currently defined  path."
        print " compute ds : ds is the distance between between planning",\
              " points in feet"

    def do_compute(self, args):

        if args:
            try:
                distance = float(args)
                self.profile = VelocityProfile(self.path, self.robot, distance)
                self.visualize.draw_velocity_profile(self.profile)
            except ValueError:
                print " Failed to parse", args
        else:
            print " Failed to parse, try \'help compute\' for more help"

    @staticmethod
    def help_show():
        print " Displays the plots. If the plots close this won't",\
              " reopen them at the moment. That requires embedding",\
              " matplotlib graphs in some gui interface and I'm lazy"

    def do_show(self, args):
        if args:
            print "Show does not take arguments"
        else:
            self.visualize.show()

    @staticmethod
    def help_save():
        print "Saves the current velocity profile."
        print " save [name] : saves to name.json"

    def do_save(self, args):
        if args:
            name = args + ".json"
        else:
            name = "profile.jsoin"
        print "saving to", name

        with open(name, 'w') as output:
            json.dump(self.profile, output, cls=ProfileEncoder)

    @staticmethod
    def help_intro():
        print "If you some how stumbled upon this program and don't know", \
              "what it is, details can be found here:", \
              " https://github.com/iqzprvagbv/path-planning/ \n"

        print "Here's a quick tl;dr on the workflow in this program:"

        print "Set the robot parameters width, maximum velocity, and", \
              " acceleration using \'robot [parameter] [value]\'\n"

        print "Add waypoints using the commands \'waypoint position", \
              " velocity accel\' If the command show is run before this", \
              " you will be able to see the path in realtime. Fixing",\
              " mistakes can be done with \'waypoint remove n\'\n"

        print "Compute the numerical velocity profile with the command",\
              " \'compute ds\' where ds is the distance between planning",\
              " points\n"

        print "If the plots look acceptable save the profile using \'",\
              "save profile_name\' which will save the file to ",\
              "profile_name.json\n"

        print "The command 'clear' will reset everything but the robot ",\
              "and let you plot again, \'quit\' closes the program, and ",\
              "\'help [command]\' will display help text and options for ",\
              "every command. Enjoy."

    @staticmethod
    def do_quit(line):
        """Quits the program"""
        if line:
            print "Quit does not take any arguments"

        return True

if __name__ == '__main__':
    PROMPT = Prompt()
    PROMPT.prompt = "> "
    PROMPT.cmdloop(INTRO_MESSAGE)
