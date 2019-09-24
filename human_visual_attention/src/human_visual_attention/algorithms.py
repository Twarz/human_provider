#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import numpy as np
import pandas as pd
import math

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import seaborn as sns

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
sigma = 1.
cov = [[sigma, 0], [0, sigma]]
x, y = np.random.multivariate_normal(mean, cov, 512).T

hits = []
for i in range(len(x)):
    hits.append(('plane', None, None, (x[i], y[i])))


nrow=1
ncol=2

radius = 2.0
gamma = 0.75
psi = 1.0
dt = 1.0
# make a list of all dataframes
fig, axes = plt.subplots(nrow, ncol, sharey=True)

ds, d_clippeds, d_norms, h_intensities = compute_intensity(center, hits, radius, gamma, psi, dt)

df_list = []
df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':ds}))
#df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':d_clippeds}))
#df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':d_norms}))
#df_list.append(pd.DataFrame({'x':x, 'y':y, 'intensities':h_intensities}))

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
for col in range(ncol):
    #axes[row].axis('equal')
    
    
    fig = df_list[count].plot.scatter(x = 'x', y = 'y', cmap=plt.get_cmap('rainbow'), c = df_list[count]['intensities'], colorbar=False, legend=False).get_figure()
    axins1 = inset_axes(fig.axes[0],
                    width="90%",  # width = 50% of parent_bbox width
                    height="5%",  # height : 5%
                    loc='lower center',
                    bbox_to_anchor=(0, 1, 1, 1),
                    bbox_transform=fig.axes[0].transAxes)

    #im1 = ax1.imshow([[1, 2], [2, 3]])
    mappable = fig.axes[0].collections[0]
    fig.axes[0].set_aspect('equal', 'box')
    fig.axes[0].set(xlim=(-3, 3), ylim=(-3, 3))
    fig.colorbar(mappable=mappable, cax=axins1, orientation="horizontal")
    axins1.xaxis.set_ticks_position("top")
    fig.savefig('plot_'+str(count)+'.pdf', bbox_inches='tight',pad_inches=0)
    count += 1

#df.reset_index().plot.scatter(x = 'x', y = 'y', cmap=plt.get_cmap('rainbow'), c = df['intensities'])
left  = 0.1  # the left side of the subplots of the figure  
right = 0.99    # the right side of the subplots of the figure  
bottom = 0.1   # the bottom of the subplots of the figure  
top = 0.9      # the top of the subplots of the figure  
wspace = 0.2   # the amount of width reserved for space between subplots,  
                # expressed as a fraction of the average axis width  
hspace = 0.2   # the amount of height reserved for space between subplots,  
                # expressed as a fraction of the average axis height 
plt.subplots_adjust(left=left, bottom=bottom, right=right, top=top, wspace=wspace, hspace=hspace)
plt.savefig('data.png', dpi=400, bbox_inches='tight',pad_inches=0)
plt.show()