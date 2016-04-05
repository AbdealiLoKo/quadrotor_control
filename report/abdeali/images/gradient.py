import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parabola(x, y):
    a = 1
    b = 1
    c = 1
    return - ( x**2 / a**2 + y**2 / b**2 ) * c

xs = np.linspace(-10 , 10, 50)
ys = np.linspace(-10 , 10, 50)

X, Y = np.meshgrid(xs, ys)

zs = np.array([parabola(x, y) for x, y in zip(np.ravel(X), np.ravel(Y))])
Z = zs.reshape(X.shape)

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')

plot_args = {'rstride': 1, 'cstride': 1, 'cmap':"gnuplot2",
             'linewidth': 0.6, 'antialiased': True, 'color': 'k'}
ax.plot_surface(X, Y, Z, **plot_args)
ax.set_xlabel('Policy parameter 1')
ax.set_ylabel('Policy parameter 2')
ax.set_zlabel('Value of policy')
ax.view_init(elev=30., azim=30)
ax.plot([0], [0], [0], 'r-', markerfacecolor='r', markeredgecolor='r', marker='o', markersize=7)
# ax.plot([0], [0], [0], markerfacecolor='r', markeredgecolor='r', marker='o', markersize=7)

plt.show()
