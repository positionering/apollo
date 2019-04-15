from __future__ import print_function

import numpy as np

import matplotlib.pyplot as plt

file  = 'mainboard.log.info.20190412-125416.32307'
        

with open(file, "r") as f:
    a = True
    b = 0
    x_offset = []
    z_offset = []
    x_offset_filtered = []
    z_offset_filtered = []
    for line in f:
        b+=1
        if b >= 10:
                  
            line = line.strip()
            line = line.split()
            if b == 599:
                break
            if a:
                a = False
                x_offset.append(line[5])
                x_offset_filtered.append(line[7])
            else:
                a = True
                z_offset.append(line[5])
                z_offset_filtered.append(line[7])
            

x = np.linspace(0,len(x_offset),len(x_offset))
z = np.linspace(0,len(z_offset),len(z_offset))

plt.plot(x, x_offset)

plt.plot(z, z_offset)

plt.plot(x, x_offset_filtered)

plt.plot(z, z_offset_filtered)

plt.savefig('filter_demo.png')
plt.show()