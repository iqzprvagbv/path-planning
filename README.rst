==============================================
Generating paths for differential drive robots
==============================================

What is this?
=============

At the moment not much, the idea is this program should take a list of waypoints, generate a smooth path through those waypoints, and then generate a list of left and right wheel velocities to follow said trajectory. All while respecting the kinematic constraints of the robot. An example of it's current output looks like:
.. image::demo.png

What's left to do?
==================

So much.
- Rotational Acceleration: at the moment no thought is put into making sure the max acceleration at each point respects rotational constraints. The rotational acceleration branch has some work done on it but it basically just keeps lowering the over all max acceleration of the robot until the wheels are never asked to do something they can't
- Output: right now all it's doing is drawing all the data, would be nice to package it into some form for robots to understand
- More Drawing: Would be nice to be able to draw the path of the left and right wheels to verify that everything looks nice
- Interactive Drawing: Dunno if I'll ever actually do this as this was just a proof of concept, but it would be nice to be able to actually define the curves in real time.
