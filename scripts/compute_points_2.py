#!/usr/bin/env python3
import numpy as np
from numpy import pi
#import tf.transformations as tft
import tf.transformations as tft
import os
os.chdir(os.path.dirname(os.path.realpath(__file__)))


def main():
    base_xyz = [-pi, 0, 0]
    # base_xyz = [-pi, -pi/2, pi/4]
    points = [{'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.5, 'alpha': 0, 'beta': 0, 'orientation':0}]
    # points += [{'type':'sphere','center':[0.5, 0, 0],
    #             'radius': 0.5, 'alpha': 0, 'beta': beta*pi/(6*6),
    #             'orientation':0} for beta in range(-6, 7)]

    points += [{'type':'sphere','center':[0.8, 0, 0],
            'radius': 0.7, 'alpha': -x*pi/(12*2), 'beta': -pi/4,
            'orientation':0} for x in range(-6, 7)]

    points += [{'type':'sphere','center':[0.4, 0, 0],
        'radius': 0.6, 'alpha': pi/2, 'beta': -x*pi/(12*2),
        'orientation':0} for x in range(-6, 7)]
    
    points += [{'type':'sphere','center':[0.6, 0, 0],
    'radius': 0.5, 'alpha': pi/2, 'beta': -x*pi/(12*2),
    'orientation':0} for x in range(-6, 7)]

    # points += [{'type':'sphere','center':[0.8, 0, 0],
    #         'radius': 0.6, 'alpha': -x*pi/(12*2), 'beta': -pi/3,
    #         'orientation':0} for x in range(-6, 7)]
    
    # points += [{'type':'sphere','center':[0.8, 0, 0],
    #     'radius': 0.6, 'alpha': -x*pi/(12*2), 'beta': -pi/4,
    #     'orientation':0} for x in range(-6, 7)]
    
    # points += [{'type':'sphere','center':[0.4, 0, 0],
    #     'radius': 0.5, 'alpha': pi/2, 'beta': -x*pi/(6*4),
    #     'orientation':0} for x in range(-6, 7)]


    
    # points = [{'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.5, 'alpha': 0, 'beta': 0, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.5, 'alpha': pi/2, 'beta': pi/12, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.5, 'alpha': pi/2, 'beta': pi/6, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.3, 'alpha': pi/2, 'beta': pi/4, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.3, 'alpha': pi/2, 'beta': -pi/12, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.3, 'alpha': pi/2, 'beta': -pi/6, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.3, 'alpha': 0, 'beta': pi/12, 'orientation':0},
    #           {'type':'sphere','center':[0.4, 0, 0] , 'radius': 0.3, 'alpha': 0, 'beta': -pi/12, 'orientation':0},
    #           {'type':'sphere','center':[1, 0.3, 0] , 'radius': 0.8, 'alpha': 0, 'beta': -pi/4, 'orientation':0},
    #           {'type':'sphere','center':[1, -0.3, 0] , 'radius': 0.8, 'alpha': 0, 'beta': -pi/4, 'orientation':0},
    # ]

    f = open('../positions/positions_calibrate_2.txt', 'w')


    for point in points:
        if point['type']=='sphere':
            x = point['center'][0]+point['radius'] * np.cos(point['alpha']) * np.sin(point['beta'])
            y = point['center'][1]+point['radius'] * np.sin(point['alpha']) * np.sin(point['beta'])
            z = point['center'][2]+point['radius'] * np.cos(point['beta'])
            beta_x = point['beta']*np.sin(point['alpha'])
            beta_y = -1*point['beta']*np.cos(point['alpha'])
            q = tft.quaternion_from_euler(base_xyz[0], base_xyz[1]+beta_y, base_xyz[2]+beta_x  + point['orientation'], 'rxyz')
            pose = np.array([x, y, z, q[0], q[1], q[2], q[3]])
        elif point['type']=='plain':
            x = point['center'][0] + point['offset'][0]
            y = point['center'][1] + point['offset'][1]
            z = point['center'][2] + point['offset'][2]
            q = tft.quaternion_from_euler(base_xyz[0], base_xyz[1], base_xyz[2]+point['orientation'], 'rxyz')
            pose = np.array([x, y, z, q[0], q[1], q[2], q[3]])
        else:
            continue
        for p in pose:
            f.write(str(p))
            f.write(' ')
        f.write('\n')
    f.close()

def sign(x):
    if x >= 0:
        return 1
    else:
        return -1
if __name__ == '__main__':
    main()