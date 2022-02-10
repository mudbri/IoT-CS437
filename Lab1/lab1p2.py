import numpy as np
from matplotlib import pyplot as plt

map = np.zeros((100, 100))

#print(map)

measurements = np.array([[45.0, 29.67], [40.0, 31.1], [35.0, 30.94], [30.0, 30.8], [25.0, 31.89], [20.0, 35.82], [15.0, 42.48], [10.0, 35.53], [5.0, 36.52], [0.0, 40.92], [-5.0, 42.62], [-10.0, 40.11], [-15.0, 71.14], [-20.0, 34.68], [-25.0, 35.31], [-30.0, 29.19], [-35.0, 27.91], [-40.0, 26.79], [-45.0, 25.85], [-50.0, 25.99]])

cols = np.hsplit(measurements,2)
xs = np.array(((np.cos(np.radians(cols[0] + 90)) * cols[1]) + 50).round(), dtype=int)

ys = np.array((np.sin(np.radians(cols[0] + 90)) * cols[1]).round(), dtype=int)

plt.scatter(xs, ys)

plt.show()

map[ys, xs] = 1

plt.imshow(map)

plt.colorbar()

plt.show()


