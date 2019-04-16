from __future__ import print_function

import numpy as np

import matplotlib.pyplot as plt

file  = 'mainboard.log.info.20190416-164030.3303'
        

with open(file, "r") as f:
    
    
    x_offset = []
    z_offset = []
    phi = []
    x_offset_filtered = []
    z_offset_filtered = []
    phi_filtered = []
    time = []

    for line in f:
       
        try:
                  
            line = line.strip()
            line = line.split()
            
            x_offset.append(line[9])
            x_offset_filtered.append(line[11])
           
            z_offset.append(line[5])
            z_offset_filtered.append(line[7])

            phi.append(line[13])
            #phi_filtered.append(line[15])

            time.append(line[15])
        except:
            pass

print(len(time))
print(len(x_offset))


plt.subplot(311)
plt.plot(time, x_offset,label="x")
plt.plot(time, x_offset_filtered,label="x_filt")
plt.legend()
plt.subplot(312)
plt.plot(time, z_offset,label="z")
plt.plot(time, z_offset_filtered,label="z_filt")
plt.legend()
plt.subplot(313)

plt.plot(time, phi, label="phi")
#plt.plot(time, phi_filtered,label="phi_filt")


plt.legend()
plt.savefig('filter_demo.png')
plt.show()
