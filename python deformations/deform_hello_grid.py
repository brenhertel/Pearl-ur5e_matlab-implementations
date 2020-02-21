#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad
import math

class point(object):
  def __init__(self, given_x=0, given_y=0):
    self.y = given_y;
    self.x = given_x; 
    
class deformation(object):
    def __init__(self, given_traj, given_initial=[], given_final=[], given_lambda=-1.0, dmp_on=True):
        self.traj = given_traj
        self.traj_length = len(self.traj)
        self.max_index = self.traj_length - 1
        if not given_initial:
            self.initial = self.traj[0]
        else:
            self.initial = given_initial
        if not given_final:
            self.final = self.traj[self.max_index]
        else:
            self.final = given_final
        if dmp_on == True:
            [self.lte, self.ja, self.dmp] = pad.perform_all_deformations(self.traj, self.initial, self.final, lmbda=given_lambda)
        else:
            [self.lte, self.ja] = pad.perform_all_deformations_no_dmp(self.traj, self.initial, self.final, lmbda=given_lambda)

def create_grid(grid_size, grid_x_dist, grid_y_dist, center):
    grid = [[point() for i in range (grid_size)] for j in range (grid_size)]
    grid_max_x = center.x + (grid_x_dist / 2)
    grid_min_x = center.x - (grid_x_dist / 2)
    grid_max_y = center.y + (grid_y_dist / 2)
    grid_min_y = center.y - (grid_y_dist / 2)
    x_vals = np.linspace(grid_min_x, grid_max_x, grid_size)
    y_vals = np.linspace(grid_min_y, grid_max_y, grid_size)
    for i in range (grid_size):
        for j in range (grid_size):
            #grid[i][j] = point(x_vals[j], y_vals[grid_size - 1 - i])
            grid[j][i].x = x_vals[j]
            grid[j][i].y = y_vals[grid_size - 1 - i]
    return grid
    

def main():
    ## Open h5 file and pull data ##
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    y_data = hello.get('resampled_y')
    x_data = np.array(x_data)
    y_data = np.array(y_data)
    hf.close()
    
    plt.figure(0)
    plt.plot(x_data, y_data)
    
    grid_size = 2
    center = point(x_data[0], y_data[0])
    grid = create_grid(grid_size, .1, .1, center)
    #for i in range (grid_size):
    #    for j in range (grid_size):
            #print('Point (%d, %d)' % (i, j))  
            #print(grid[i][j].x)
            #print(grid[i][j].y)
            #plt.plot(grid[i][j].x, grid[i][j].y, 'k+')
    #print('Start: (%f, %f)' % (x_data[0], y_data[0]))
    #print('Middle Point: (%f, %f)' % (grid[2][2].x, grid[2][2].y))  
    #plt.show()
    
    dmp_end_off = 0;
    
    #base_deform_x = deformation(x_data)
    #base_deform_y = deformation(y_data)
    #plt.plot(base_deform_x.traj, base_deform_y.traj)
    #plt.plot(base_deform_x.lte, base_deform_y.lte)
    #plt.plot(base_deform_x.ja, base_deform_y.ja)
    #plt.plot(base_deform_x.dmp, base_deform_y.dmp)
    #plt.show()
    print(filename + '_grid' + str(grid_size) + '.h5')
    fp = h5py.File (filename + '_grid' + str(grid_size) + '.h5', 'w')
    dset_name = 'hello'
    
    for i in range (grid_size):
        for j in range (grid_size):
            base_deform_x = deformation(x_data, grid[i][j].x)
            base_deform_y = deformation(y_data, grid[i][j].y)
            ax = plt.subplot2grid((grid_size, grid_size), (i, j))
            ax.plot(base_deform_x.traj, base_deform_y.traj, 'b')
            ax.plot(base_deform_x.lte, base_deform_y.lte, 'g')
            ax.plot(base_deform_x.ja, base_deform_y.ja, 'r')
            ax.plot(base_deform_x.dmp, base_deform_y.dmp, 'm')
            fp.create_dataset(dset_name + '/original/(' + str(i) + ', ' + str(j) + ')/x', data=base_deform_x.traj)
            fp.create_dataset(dset_name + '/original/(' + str(i) + ', ' + str(j) + ')/y', data=base_deform_y.traj)
            fp.create_dataset(dset_name + '/lte/(' + str(i) + ', ' + str(j) + ')/x', data=base_deform_x.lte)
            fp.create_dataset(dset_name + '/lte/(' + str(i) + ', ' + str(j) + ')/y', data=base_deform_y.lte)
            fp.create_dataset(dset_name + '/ja/(' + str(i) + ', ' + str(j) + ')/x', data=base_deform_x.ja)
            fp.create_dataset(dset_name + '/ja/(' + str(i) + ', ' + str(j) + ')/y', data=base_deform_y.ja)
            fp.create_dataset(dset_name + '/dmp/(' + str(i) + ', ' + str(j) + ')/x', data=base_deform_x.dmp)
            fp.create_dataset(dset_name + '/dmp/(' + str(i) + ', ' + str(j) + ')/y', data=base_deform_y.dmp)
            #plt.plot(base_deform_x.traj, base_deform_y.traj, 'b')
            #plt.plot(base_deform_x.lte, base_deform_y.lte, 'g')
            #plt.plot(base_deform_x.ja, base_deform_y.ja, 'r')
            #plt.plot(base_deform_x.dmp, base_deform_y.dmp, 'm')
            #plt.show()
            diff_x = abs(base_deform_x.traj[len(base_deform_x.traj) - 1] - base_deform_x.dmp[len(base_deform_x.dmp) - 1])
            diff_y = abs(base_deform_y.traj[len(base_deform_y.traj) - 1] - base_deform_y.dmp[len(base_deform_y.dmp) - 1])
            diff_xy = math.sqrt(diff_x**2 + diff_y**2)
            dmp_end_off = dmp_end_off + diff_xy;
    normalized_dmp_end_off = dmp_end_off / grid_size
    print('normalized_dmp_end_off: %f' % (normalized_dmp_end_off))
    fp.close()
    plt.show()
    

if __name__ == '__main__':
    main()