import spline as s

import matplotlib.pyplot as plt

w1 = s.Waypoint((0,0),(0,20),(1,0))
w2 = s.Waypoint((5,5),(0,20),(-1,0))

x = s.from_waypoints(w1,w2)

x.draw(plt)

plt.axis('equal')
plt.show()
