==============================================
Generating paths for differential drive robots
==============================================

What is this?
=============

At the moment not much, the idea is this program should take a list of waypoints, generate a smooth path through those waypoints, and then generate a list of left and right wheel velocities to follow said trajectory. All while respecting the kinematic constraints of the robot. An example of it's current output looks like:

..  image:: https://raw.githubusercontent.com/iqzprvagbv/path-planning/master/demo.png

What's left to do?
==================

So much.

- Rotational Acceleration: at the moment rotational acceleration concerns are met clunkily. Basically the code just keeps lowering the over all max acceleration of the robot until the wheels are never asked to do something they can't do. This is suboptimal as it lowers the acceleration in places where it doesn't need to, and it stops as soon as the acceleration works resulting in potentially lower acceleration than is possible.
- Output: right now all it's doing is drawing all the data, would be nice to package it into some form for robots to understand
- Interactive Drawing: Dunno if I'll ever actually do this as this was just a proof of concept, but it would be nice to be able to actually define the curves in real time.
