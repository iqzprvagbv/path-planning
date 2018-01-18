===============
Path Generation
===============
------------------------------------------
Generating paths differential drive robots
------------------------------------------

What is this?
=============

At the moment not much, the idea is this program should take a list of waypoints and generate a list of left and right velocities for the robot to follow a path which goes through each of the waypoints smoothly. I plan to fill this readme out better when it's actually usable, as it stands it's basically a pet project.

What's left to do?
==================

So much. Currently the program will draw all the data for humans to verify but won't output anything for a robot to consume. Moreover it doesn't take into account the rotational velocity constraints when generating the velocity profile. Even the drawing could use work, it would be nice if it showed the width of the robot, as it stands it only draws the center points motion. 
