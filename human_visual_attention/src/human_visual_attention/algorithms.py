#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import numpy as np
import pandas as pd
import math

import matplotlib.pyplot as plt


def compute_intensity(hit_center, hits, max_radius, gamma, penalty, dt):
    ds = []
    d_clippeds = []
    d_norms = []
    #intensities = []
    h_intensities = []
    for h in hits:
        h_object = h[0]
        if not h_object:
            continue
        
        h_pos = np.array(h[3])
        d = math.sqrt(np.sum((hit_center[1] - h_pos)**2)) # Euler distance
        ds.append(d)

        d_clipped = min(d, max_radius)
        d_clippeds.append(d_clipped)
        d_norm = d_clipped / max_radius
        d_norms.append(d_norm)

        h_intensity = 1.0 - d_norm
        #intensities.append(h_intensity)
        h_intensity = h_intensity * dt
        h_intensity = h_intensity * penalty

        if h_object != hit_center[0]:
            h_intensity = h_intensity * gamma
        h_intensities.append(h_intensity)

        

    
    return ds, d_clippeds, d_norms, h_intensities


np.random.seed(1234324)
center = ('plane', np.array([0., 0.]))
mean = [0, 0]
cov = [[1, 0], [0, 1]]
x, y = np.random.multivariate_normal(mean, cov, 512).T

hits = []
for i in range(len(x)):
    hits.append(('plane', None, None, (x[i], y[i])))


nrow=5
ncol=2

radius = [1, 4]
# make a list of all dataframes 
fig, axes = plt.subplots(nrow, ncol)
for col, p in enumerate([1.0, 0.3]):

    ds, d_clippeds, d_norms, h_intensities = compute_intensity(center, hits, radius[col], 0.75, p, 1)

    df_list = []
    df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':ds}))
    df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':d_clippeds}))
    df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':d_norms}))
    df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':h_intensities}))

    selected_x =[]
    selected_y =[]
    selected_i =[]

    for xi, yi, ii in zip(x, y, h_intensities):
        if ii > 0.0:
            selected_x.append(xi)
            selected_y.append(yi)
            selected_i.append(ii)


    df_list.append(pd.DataFrame({'x':selected_x, 'y':selected_y, 'intensities':selected_i}))

    # plot counter
    count=0
    for row in range(nrow):
        #axes[row].axis('equal')
        axes[row, col].set_aspect('equal', 'box')
        axes[row, col].set(xlim=(-3, 3), ylim=(-3, 3))
        
        df_list[count].plot.scatter(x = 'x', y = 'y', cmap=plt.get_cmap('rainbow'), c = df_list[count]['intensities'], ax=axes[row, col])
        count += 1
#df.reset_index().plot.scatter(x = 'x', y = 'y', cmap=plt.get_cmap('rainbow'), c = df['intensities'])
left  = 0.25  # the left side of the subplots of the figure
right = 0.25   # the right side of the subplots of the figure
bottom = 0.1   # the bottom of the subplots of the figure
top = 2.5      # the top of the subplots of the figure
wspace = 0.2   # the amount of width reserved for blank space between subplots
hspace = 0.2   # the amount of height reserved for white space between subplots
plt.subplots_adjust(left=None, bottom=bottom, right=None, top=top, wspace=None, hspace=None)
plt.savefig('data.png', dpi=400, bbox_inches='tight',pad_inches=0)
plt.show()