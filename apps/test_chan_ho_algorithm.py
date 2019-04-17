#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import chan_ho_algorithm

# Fixing random state for reproducibility
#np.random.seed(19680801)
#x_t = np.random.rand(1)
#y_t = np.random.rand(1)

x_r = [0,1,2,0]
y_r = [0,1,0,1]

x_t = 1.2
y_t = 0

d = list()
pos_rx = list()
for rx_id in range(len(x_r)):
    pos_rx.append([x_r[rx_id],y_r[rx_id]])
    if 0 != rx_id:
        # Calculate TDOA and add some measurement noise
        d.append(np.sqrt((x_r[rx_id]-x_t)**2 + (y_r[rx_id]-y_t)**2) \
            - np.sqrt((x_r[0]-x_t)**2 + (y_r[0]-y_t)**2) + np.random.rand(1)/10)
print("pos_rx",pos_rx)
print("d",d)

xy = chan_ho_algorithm.locate(pos_rx, d)
print("xy",xy)

plt.scatter(x_r, y_r)
plt.scatter(x_t, y_t)
plt.scatter(xy[0], xy[1])
plt.show()
