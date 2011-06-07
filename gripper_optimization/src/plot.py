#!/usr/bin/python

from numpy import *
import sys
import pylab as p
import mpl_toolkits.mplot3d.axes3d as p3
import pdb
filelines = file(sys.argv[1]).readlines()

inner_count = 131
outer_count = 102

dof0 = []
dof0neg = []
dof1 = []
magn = zeros([inner_count, outer_count]) 
force = zeros([inner_count, outer_count]) 
resid_x = zeros([inner_count, outer_count]) 
resid_y = zeros([inner_count, outer_count]) 

xind = 0
yind = 0

offset=2 + 4*2

max_resid_norm = 0
min_resid_norm = -1

scale_arrows = 1
max_scale = 10 * 1.0e6

show_magnitudes = 0
scale_magnitude = 1
max_magnitude = 10 * 1.0e6

show_forces = 0

if (len(sys.argv) > 2):
    offset = 2 + 4*int(sys.argv[2])
    if (sys.argv[2] == '0'):
        show_forces = 1
        show_magnitudes = 1

for line in filelines:
    floats = [float(number) for number in line.split(',')]
    dof0.append(floats[0])
    dof0neg.append(-floats[0])
    dof1.append(floats[1])
    try:
        force[yind,xind] = floats[offset+0]
        magn[yind,xind] = floats[offset+1]
        if scale_magnitude and magn[yind,xind] > max_magnitude:
            magn[yind,xind] = max_magnitude
        resid_x[yind,xind] = floats[offset+2]
        resid_y[yind,xind] = floats[offset+3]
        resid_norm = sqrt(resid_x[yind,xind] * resid_x[yind,xind] + resid_y[yind,xind] * resid_y[yind,xind])
        if resid_norm > max_resid_norm:
            max_resid_norm = resid_norm
        if min_resid_norm < 0 or resid_norm < min_resid_norm:
            min_resid_norm = resid_norm
        if scale_arrows and resid_norm > max_scale:
            resid_x[yind,xind] *= max_scale / resid_norm;
            resid_y[yind,xind] *= max_scale / resid_norm;
    except:
        pdb.set_trace()
    yind +=1
    if yind >= inner_count:
        yind = 0
        xind += 1

print max_resid_norm
print min_resid_norm

xset = sort(list(set(dof0))).tolist()
yset = sort(list(set(dof1))).tolist()

samples0 = []
samples1 = []

dof0min = yset[0]
dof0max = yset[inner_count-1]
dof1min = xset[0]
dof1max = xset[outer_count-1]
sampling = 11
step0 = (dof0max - dof0min) / (sampling - 1)
step1 = (dof1max - dof1min) / (sampling - 1)
#for i in range(0, sampling):
#    for j in range(0, sampling):
#        samples0.append( dof0min + i*step0 )
#        samples1.append( dof1min + j*step1 )

numEnvelopingPoses = 7
envelopingMin = 30.0
envelopingMax = 67.0
step = (envelopingMax - envelopingMin) / (numEnvelopingPoses - 1)
M_PI = 3.141592653
for i in range(0, numEnvelopingPoses):
    d0 = envelopingMin + i * step
    d1 = 80.0 - 2*d0
    samples1.append( d0 * M_PI / 180.0 )
    samples0.append( d1 * M_PI / 180.0 )

numFingertipPoses = 11
fingertipMin =  2.5
fingertipMax = 95.0
step = (fingertipMax - fingertipMin) / (numFingertipPoses - 1)
for i in range(0, numFingertipPoses):
    d0 = fingertipMin + i * step
    d1 = -d0
    samples1.append( d0 * M_PI / 180.0 )
    samples0.append( d1 * M_PI / 180.0 )


p.quiver(xset[::3], yset[::3], resid_x[::3,::3], resid_y[::3,::3])
p.plot(dof0, dof0neg, 'r')
p.plot(samples1, samples0, 'ro')

if show_magnitudes:
    p.figure()
    p.contourf(xset,yset,magn,20)
    p.plot(dof0, dof0neg, 'w')

if show_forces:
    p.figure()
    p.contourf(xset,yset,force,20)
    p.plot(dof0, dof0neg, 'w')

p.show()
