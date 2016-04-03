
# This is a snippet. This is meant to be copied into ipython or python shell
# for easy figure creation.

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Get X from ipython environment
# Get Y from ipython environment

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


ax.plot(X, Y, 'r-',
        markerfacecolor='r', markeredgecolor='r', marker='o', markersize=2)
# ax.plot([0], [0], [0], markerfacecolor='r', markeredgecolor='r', marker='o', markersize=7)

plt.show()
