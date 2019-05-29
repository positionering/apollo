import os
import errno
import numpy as np
import matplotlib.pyplot as plt

FIFO = '/tmp/plot'

try:
    os.mkfifo(FIFO)
except OSError as oe:
    if oe.errno != errno.EEXIST:
        raise
x=[]
y=[]
while True:
    print("Opening FIFO...")
    with open(FIFO) as fifo:
        print("FIFO opened")
        plt.ion()
        plt.show()
        
        while True:  
              
            while True:
                data = fifo.read()
                if len(data) != 0:
                    break
            
            qwe = data.split(",")[0]
             
            print('Read: "{0}"'.format(qwe))
            

            x.append(float(qwe.split(" ")[0]))
            y.append(float(qwe.split(" ")[1]))
            plt.cla()
            #plt.axis([0, 10, 0, 10])
            plt.plot(x, y)
            plt.draw()
            plt.pause(0.0000001)








