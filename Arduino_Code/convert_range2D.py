#!/usr/bin/env python

import math,sys

file = sys.argv[1]

#file = open("/Users/jscholz/Documents/Arduino/rangeloop_raw.dat")
print "Converting ", file

for i in file.readlines():
    #print i
    angle = i.strip("\n").split(" ")[0]
    dist = i.strip("\n").split(" ")[1]
#    print angle,dist
#    x = (500 - float(dist)) * math.cos(float(angle) * math.pi/180)
#    y = (500 - float(dist)) * math.sin(float(angle) * math.pi/180)
    x = (1/float(dist)+0.42) * math.cos(float(angle) * math.pi/180)
    y = (1/float(dist)+0.42) * math.sin(float(angle) * math.pi/180)
    print x, y

    
