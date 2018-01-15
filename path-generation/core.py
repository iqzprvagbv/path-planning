from path    import from_waypoints
from profile import VelocityProfile
from robot   import Robot
from spline  import Waypoint

# Width of the robot in feet
robot_width = 2
# Max Velocity of robot in fps
max_velocity = 1
# Max acceleration of robot in fps^2
max_acceleration = 10

# Way points for the curve to hit
# Format: Waypoint(position, velocity vector, acceleration vector)
waypoints = []
waypoints.append(Waypoint((0,0),(0,10),(1,0)))
waypoints.append(Waypoint((5,5),(0,10),(-1,0)))
waypoints.append(Waypoint((0,10),(0,10),(1,0)))

# Distance between planning points in feet
ds = 0.1

path = from_waypoints(waypoints)
robot = Robot(robot_width,max_velocity,max_acceleration)
vp = VelocityProfile(path,robot,ds)
vp.draw()
