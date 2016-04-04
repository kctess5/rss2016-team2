"""
Print out some example paths.
You can eyeball them to see if they look ok-ish.
"""

import numpy as np
import local_costmap

pg = local_costmap.PathGenerator()

print "Zero path"
print pg.generate_path(0.).waypoints

print "Slight path"
print pg.generate_path(0.01).waypoints

print pg.generate_path(0.01).waypoints.shape
print pg.generate_path(0.01).waypoints[6,1]
print pg.generate_path(0.01).waypoints[0]

# Graph something.
import matplotlib.pyplot as plt
path_to_plot = pg.generate_path(np.deg2rad(4))
plt.plot(path_to_plot.waypoints[:,0],
         path_to_plot.waypoints[:,1],
         'ro')
plt.show()
