import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad
import math
import deform_hello_grid
import optimize_ja
import similaritymeasures
from scipy.spatial.distance import directed_hausdorff
from matplotlib.colors import LogNorm
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp2d
import gradient_plotting
import os
from scipy.interpolate import RegularGridInterpolator
import seaborn as sns; sns.set()

class point(object):
  def __init__(self, given_x=None, given_y=None, given_z=None):
    self.y = given_y
    self.x = given_x 
    self.z = given_z

class traj(object):
  def __init__(self, given_traj=[]):
    self.traj = given_traj

class val(object):
  def __init__(self, value=0.0):
    self.val = value
    
def my_map(x, in_min, in_max, out_min, out_max): #arduino's map function
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    
def downsample_1d(traj, n=100):
    npts = np.linspace(0, len(traj) - 1, n)
    out = np.zeros((n))
    for i in range (n):
        out[i] = traj[int(npts[i])]
    return out

def get_euclidian_dist(x1, y1, x2, y2):
    return ((x2-x1)**2 + (y2-y1)**2)**0.5

def get_total_dist(x, y):
    ttl = 0
    for i in range (len(x) - 2):
        ttl = ttl + get_euclidian_dist(x[i], y[i], x[i+1], y[i+1])
    return ttl
    
def get_start_end_dist(x, y):
    return get_euclidian_dist(x[0], y[0], x[len(x) - 1], y[len(y) - 1])
    
class GSM(object):
  def __init__(self):
    self.n_algs = 0
    self.n_metrics = 0
    self.n_dims = 0
    self.traj_len = 0
    self.org_x = []
    self.org_y = []
    self.org_z = []
    self.algs = []
    self.alg_names = []
    self.metrics = []
    self.metric_names = []
    self.is_dissims = []
    self.f_size = 32
    
  def add_traj_dimension(self, traj, dim=''):
    if (self.traj_len != 0 and len(traj) != self.traj_len):
        print('Length of given trajectories must agree! Trajectory not added.')
        return
    if (dim == 'x'):
        self.org_x = traj
    elif (dim == 'y'):
        self.org_y = traj
    elif (dim == 'z'):
        self.org_z = traj
    else:
        if (self.org_x == []):
            self.org_x = traj
        elif (self.org_y == []):
            self.org_y = traj
        elif (self.org_z == []):
            self.org_z = traj
        else:
            print('Error: trajectory not added. No available trajectory slots and no dimension specified')
            return
    self.traj_len = len(traj)
    self.n_dims = self.n_dims + 1
        
  def add_deform_alg(self, alg, name=''):
    self.algs.append(alg)
    self.alg_names.append(name)
    self.n_algs = self.n_algs + 1
    
  def add_sim_metric(self, metric, name='', is_dissim=False):
    self.metrics.append(metric)
    self.metric_names.append(name)
    self.n_metrics = self.n_metrics + 1
    self.is_dissims.append(is_dissim)
    
  def create_grid(self, given_grid_size, dists, disp=False):
    if (np.size(self.org_x) == 0):
        print('WARNING: No trajectories given')
    self.grid_size = given_grid_size
    if (self.n_dims >= 1):
        center = point(self.org_x[0])
        grid_max_x = center.x + (dists[0] / 2)
        grid_min_x = center.x - (dists[0] / 2)
        self.x_vals = np.linspace(grid_min_x, grid_max_x, self.grid_size)
        #print(np.reshape(self.x_vals, (self.grid_size)))
    if (self.n_dims >= 2):
        center = point(self.org_x[0], self.org_y[0])
        grid_max_y = center.y + (dists[1] / 2)
        grid_min_y = center.y - (dists[1] / 2)
        self.y_vals = np.linspace(grid_min_y, grid_max_y, self.grid_size)
    if (self.n_dims >= 3):
        center = point(self.org_x[0], self.org_y[0], self.org_z[0])
        grid_max_z = center.z + (dists[2] / 2)
        grid_min_z = center.z - (dists[2] / 2)
        self.z_vals = np.linspace(grid_min_z, grid_max_z, self.grid_size)
    if (self.n_dims == 1):
        self.grid = [point() for i in range (self.grid_size)]
        #self.grid_x = np.meshgrid(x_vals)
        self.grid_deforms_x = [[traj() for n in range (self.n_algs)] for i in range (self.grid_size)]
        self.grid_similarities = [[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)]
        for i in range (self.grid_size):
            self.grid[i].x = self.x_vals[i]
        if (disp == True):
            for i in range (self.grid_size):
                print('X: %f' % (self.grid[i].x))
    if (self.n_dims == 2):
        self.grid = [[point() for i in range (self.grid_size)] for j in range (self.grid_size)]
        #self.grid_x, self.grid_y = np.meshgrid(x_vals, y_vals)
        self.grid_deforms_x = [[[traj() for n in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        self.grid_deforms_y = [[[traj() for n in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        self.grid_similarities = [[[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                self.grid[j][i].x = self.x_vals[j]
                self.grid[j][i].y = self.y_vals[self.grid_size - 1 - i]
        if (disp == True):
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    print('X: %f, Y: %f' % (self.grid[i][j].x, self.grid[i][j].y))
    if (self.n_dims == 3):
        self.grid = [[[point() for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        #self.grid_x, self.grid_y, self.grid_z = np.meshgrid(x_vals, y_vals, z_vals)
        self.grid_deforms_x = [[[[traj() for n in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        self.grid_deforms_y = [[[[traj() for n in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        self.grid_deforms_z = [[[[traj() for n in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)] 
        self.grid_similarities = [[[[[val() for m in range (self.n_algs)]for n in range (self.n_metrics)]  for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)] 
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for k in range (self.grid_size):
                    self.grid[j][i][k].x = self.x_vals[j]
                    self.grid[j][i][k].y = self.y_vals[self.grid_size - 1 - i]
                    self.grid[j][i][k].z = self.z_vals[k]
        if (disp == True):
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        print('X: %f, Y: %f, Z: %f' % (self.grid[i][j][k].x, self.grid[i][j][k].y, self.grid[i][j][k].z))
    
  def deform_traj(self, plot=False):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    print(np.shape(self.grid))
    print(np.shape(self.grid_deforms_x))
    for n in range (self.n_algs):
        if self.n_dims == 1:
            for i in range (self.grid_size):
                self.grid_deforms_x[i][n].traj = self.algs[n](self.org_x, self.grid[i].x)
                if plot == True:
                    ax = plt.subplot2grid((self.grid_size), (i))
                    ax.plot(self.grid_deforms_x[i][n].traj, colors[n])
                    ax.plot(self.org_x, 'k')
                    plt.xticks([])
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    print('i: %d, j: %d, n: %d' % (i, j, n))
                    self.grid_deforms_x[i][j][n].traj = self.algs[n](self.org_x, self.grid[i][j].x)
                    self.grid_deforms_y[i][j][n].traj = self.algs[n](self.org_y, self.grid[i][j].y)
                    #print('start: (%f, %f)' % (self.grid_deforms_x[i][j][n].traj[0], self.grid_deforms_y[i][j][n].traj[0]))
                    if plot == True:
                        ax = plt.subplot2grid((self.grid_size, self.grid_size), (i, j))
                        ax.plot(self.grid_deforms_x[i][j][n].traj, self.grid_deforms_y[i][j][n].traj, colors[n])
                        ax.plot(self.org_x, self.org_y, 'k')
                        plt.xticks([])
                        plt.yticks([])
            plt.show()
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        self.grid_deforms_x[i][j][k][n].traj = self.algs[n](self.org_x, self.grid[i][j][k].x)
                        self.grid_deforms_y[i][j][k][n].traj = self.algs[n](self.org_y, self.grid[i][j][k].y)
                        self.grid_deforms_z[i][j][k][n].traj = self.algs[n](self.org_z, self.grid[i][j][k].z)
                        if plot == True:
                            print('Plotting a grid of 3D plots is too hard!')
    plt.show()
    plt.close('all')
    
  def calc_metrics(self, d_sample=True, n_dsample=100):
    for n in range (self.n_metrics):
        #A = np.zeros((self.grid_size, self.grid_size))
        metric_max = None
        metric_min = None
        for m in range (self.n_algs):
            if d_sample == True:
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities[i][n][m].val = self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][m].traj, n_dsample))
                        if (metric_max == None or self.grid_similarities[i][n][m].val > metric_max):
                            metric_max = self.grid_similarities[i][n][m].val
                        if (metric_min == None or self.grid_similarities[i][n][m].val < metric_min):
                            metric_min = self.grid_similarities[i][n][m].val
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            #print('org_x')
                            #print(self.org_x)
                            #print('deform_x')
                            #print(self.grid_deforms_x[i][j][m].traj)
                            #print('org_y')
                            #print(self.org_y)
                            #print('deform_y')
                            #print(self.grid_deforms_y[i][j][m].traj)
                            #print('i: %d, j: %d' % (i, j))
                            self.grid_similarities[i][j][n][m].val = self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][j][m].traj, n_dsample), downsample_1d(self.org_y, n_dsample), downsample_1d(self.grid_deforms_y[i][j][m].traj, n_dsample))
                            #print(self.grid_similarities[i][j][n][m].val)
                            if (metric_max == None or self.grid_similarities[i][j][n][m].val > metric_max):
                                metric_max = self.grid_similarities[i][j][n][m].val
                            if (metric_min == None or self.grid_similarities[i][j][n][m].val < metric_min):
                                metric_min = self.grid_similarities[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities[i][j][k][n][m].val = self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][j][k][m].traj, n_dsample), downsample_1d(self.org_y, n_dsample), downsample_1d(self.grid_deforms_y[i][j][k][m].traj, n_dsample), downsample_1d(self.org_z, n_dsample), downsample_1d(self.grid_deforms_z[i][j][k][m].traj, n_dsample))
                                if (metric_max == None or self.grid_similarities[i][j][k][n][m].val > metric_max):
                                    metric_max = self.grid_similarities[i][j][k][n][m].val
                                if (metric_min == None or self.grid_similarities[i][j][k][n][m].val < metric_min):
                                    metric_min = self.grid_similarities[i][j][k][n][m].val
            else:
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities[i][n][m].val = self.metrics[n](self.org_x, self.grid_deforms_x[i][m].traj)
                        if (metric_max == None or self.grid_similarities[i][n][m].val > metric_max):
                            metric_max = self.grid_similarities[i][n][m].val
                        if (metric_min == None or self.grid_similarities[i][n][m].val < metric_min):
                            metric_min = self.grid_similarities[i][n][m].val
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            self.grid_similarities[i][j][n][m].val = self.metrics[n](self.org_x, self.grid_deforms_x[i][j][m].traj, self.org_y, self.grid_deforms_y[i][j][m].traj)
                            print(self.grid_similarities[i][j][n][m].val)
                            if (metric_max == None or self.grid_similarities[i][j][n][m].val > metric_max):
                                metric_max = self.grid_similarities[i][j][n][m].val
                            if (metric_min == None or self.grid_similarities[i][j][n][m].val < metric_min):
                                metric_min = self.grid_similarities[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities[i][j][k][n][m].val = self.metrics[n](self.org_x, self.grid_deforms_x[i][j][k][m].traj, self.org_y, self.grid_deforms_y[i][j][k][m].traj, self.org_z, self.grid_deforms_z[i][j][k][m].traj)
                                if (metric_max == None or self.grid_similarities[i][j][k][n][m].val > metric_max):
                                    metric_max = self.grid_similarities[i][j][k][n][m].val
                                if (metric_min == None or self.grid_similarities[i][j][k][n][m].val < metric_min):
                                    metric_min = self.grid_similarities[i][j][k][n][m].val
        if (self.is_dissims[n] == True):
            for m in range (self.n_algs):
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities[i][n][m].val = my_map(self.grid_similarities[i][n][m].val, metric_min, metric_max, 1, 0)
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            #print('n: %s m: %s i: %d j: %d org: %f' % (self.metric_names[n], self.alg_names[m], i, j, self.grid_similarities[i][j][n][m].val))
                            self.grid_similarities[i][j][n][m].val = my_map(self.grid_similarities[i][j][n][m].val, metric_min, metric_max, 1, 0)
                            #print('n: %s m: %s i: %d j: %d new: %f' % (self.metric_names[n], self.alg_names[m], i, j, self.grid_similarities[i][j][n][m].val))
                            #A[i][j] = self.grid_similarities[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities[i][j][k][n][m].val = my_map(self.grid_similarities[i][j][k][n][m].val, metric_min, metric_max, 1, 0)
                if (m == 3):
                    print(self.get_array_of_sim_metrics(n, m))
        else:
            for m in range (self.n_algs):
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities[i][n][m].val = my_map(self.grid_similarities[i][n][m].val, metric_min, metric_max, 0, 1)
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            self.grid_similarities[i][j][n][m].val = my_map(self.grid_similarities[i][j][n][m].val, metric_min, metric_max, 0, 1)
                            #A[i][j] = self.grid_similarities[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities[i][j][k][n][m].val = my_map(self.grid_similarities[i][j][k][n][m].val, metric_min, metric_max, 0, 1)
                
    #print(A)
    #gradient_plotting.gradient_map_show(A, 'test', 0, 1)
    
  def save_results(self, filename='unspecified.h5'):
    fp = h5py.File(filename, 'w')
    dset_name = 'GSM'
    fp.create_dataset(dset_name + '/grid_sz', data=self.grid_size)
    for m in range (self.n_algs):
            if self.n_dims == 1:
                for i in range (self.grid_size):
                    fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ')/x', data=self.grid_deforms_x[i][m].traj)
            if self.n_dims == 2:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ')/x', data=self.grid_deforms_x[i][j][m].traj)
                        fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ')/y', data=self.grid_deforms_y[i][j][m].traj)
            if self.n_dims == 3:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/x', data=self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/y', data=self.grid_deforms_y[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/z', data=self.grid_deforms_z[i][j][k][m].traj)
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            A = self.get_array_of_sim_metrics(n, m)
            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/' + self.metric_names[n], data=A)
    fp.close()
    
  def read_from_h5(self, filename):
    fp = h5py.File(filename, 'r')
    dset_name = 'GSM'
    dset = fp.get(dset_name)
    gs = dset.get('grid_sz')
    gs = np.array(gs)
    self.create_grid(gs, [0, 0, 0])
    for m in range (self.n_algs):
        alg_name = self.alg_names[m]
        alg = dset.get(alg_name)
        if self.n_dims == 1:
            for i in range (self.grid_size):
                deform_name = '(' + str(i) + ')'
                deform = alg.get(deform_name)
                x = deform.get('x')
                self.grid_deforms_x[i][m].traj = np.array(x)
                self.x_vals[i] = self.grid_deforms_x[i][m].traj[0]
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    deform_name = '(' + str(i) + ', ' + str(j) + ')'
                    deform = alg.get(deform_name)
                    x = deform.get('x')
                    self.grid_deforms_x[i][j][m].traj = np.array(x)
                    y = deform.get('y')
                    self.grid_deforms_y[i][j][m].traj = np.array(y)
                    if (j == 0):
                        self.x_vals[i] = self.grid_deforms_x[i][j][m].traj[0]
                    if (i == 0):
                        self.y_vals[self.grid_size - 1 - j] = self.grid_deforms_y[i][j][m].traj[0]
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        deform_name = '(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')'
                        deform = alg.get(deform_name)
                        x = deform.get('x')
                        self.grid_deforms_x[i][j][k][m].traj = np.array(x)
                        x = deform.get('y')
                        self.grid_deforms_y[i][j][k][m].traj = np.array(y)
                        z = deform.get('z')
                        self.grid_deforms_y[i][j][k][m].traj = np.array(z)
                        if (j == 0 and k == 0):
                            self.x_vals[i] = self.grid_deforms_x[i][j][k][m].traj[0]
                        if (i == 0 and k == 0):
                            self.y_vals[j] = self.grid_deforms_y[i][j][k][m].traj[0]
                        if (i == 0 and j == 0):
                            self.y_vals[k] = self.grid_deforms_z[i][j][k][m].traj[0]
        for n in range (self.n_metrics):
            metric_name = self.metric_names[n]
            metric = alg.get(metric_name)
            metric = np.array(metric)
            if self.n_dims == 1:
                for i in range (self.grid_size):
                    self.grid_similarities[i][n][m].val = metric[i]
            if self.n_dims == 2:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        self.grid_similarities[i][j][n][m].val = metric[i][j]
            if self.n_dims == 3:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            self.grid_similarities[i][j][k][n][m].val = metric[i][j][k]
    fp.close()
  
  def plot_gradients(self, mode='save', filepath=''):
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            if self.n_dims == 1:
                A = self.get_array_of_sim_metrics(n, m)
                if (mode == 'save'):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')                                      
            if self.n_dims == 2:
                A = self.get_array_of_sim_metrics(n, m)
                if (mode == 'save'):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')   
            if self.n_dims == 3:
                A = self.get_array_of_sim_metrics(n, m)
                if (mode == save):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient') 
    plt.close('all')

  def plot_heatmap(self, mode='save', filepath=''):
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            name = self.alg_names[m] + ' ' + self.metric_names[n] + ' Heatmap'
            print(name)
            A = self.get_array_of_sim_metrics(n, m)
            ax = sns.heatmap(np.transpose(A), annot=False)
            plt.xticks([])
            plt.yticks([])
            if (mode == 'save'):
                plt.savefig(filepath + name + '.png')
            else:            
                plt.show() 
            plt.close('all')

  def plot_strongest_gradients(self, mode='save', filepath=''):
    for n in range (self.n_metrics):
        if self.n_dims == 1:
            A = np.zeros((self.grid_size))
            for i in range (self.grid_size):
                max_s = None
                for m in range (self.n_algs):
                    if (max_s == None or max_s < self.grid_similarities[i][n][m].val):
                        max_s = self.grid_similarities[i][n][m].val
                        max_m = m
                A[i] = max_m
            B = self.convert_num_to_rgb(A)
            im = plt.imshow(np.transpose(B), vmin=0, vmax=1)
            plt.xticks([])
            plt.title(self.metric_names[n] + 'Comparison', fontsize=self.f_size)
            if (mode == 'save'):
                plt.savefig(filepath + self.metric_names[n] + 'Comparison' + '.png')
            else:
                plt.show()                                     
        if self.n_dims == 2:
            A = np.zeros((self.grid_size, self.grid_size))
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    max_s = None
                    for m in range (self.n_algs):
                        if (max_s == None or max_s < self.grid_similarities[i][j][n][m].val):
                            max_s = self.grid_similarities[i][j][n][m].val
                            max_m = m
                    A[i][j] = max_m
            B = self.convert_num_to_rgb(A)
            im = plt.imshow(np.transpose(B), vmin=0, vmax=1)
            plt.xticks([])
            plt.yticks([])
            plt.title(self.metric_names[n] + 'Comparison', fontsize=self.f_size)
            if (mode == 'save'):
                plt.savefig(filepath + self.metric_names[n] + 'Comparison' + '.png')
            else:
                plt.show()           
        if self.n_dims == 3:
            print('Gradient in 3D too difficult to show')
    plt.close('all')
        
  def convert_num_to_rgb(self, A):
    #colors = ['r', 'g', 'b', 'c', 'm', 'y']
    if self.n_dims == 1:
        B = np.zeros((self.grid_size, 3))
        for i in range (self.grid_size):
            if A[i] == 0:
                B[i][0] = 255
            elif A[i] == 1:
                B[i][1] = 255
            elif A[i] == 2:
                B[i][2] = 255
            elif A[i] == 3:
                B[i][0] = 255
                B[i][1] = 255
            elif A[i] == 4:
                B[i][0] = 255
                B[i][2] = 255
            elif A[i] == 5:
                B[i][1] = 255
                B[i][2] = 255
            else:
                print('Too many algorithms to represent color')
    if self.n_dims == 2:
        B = np.zeros((self.grid_size, self.grid_size, 3))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                if A[i][j] == 0:
                    B[i][j][0] = 255
                elif A[i][j] == 1:
                    B[i][j][1] = 255
                elif A[i][j] == 2:
                    B[i][j][2] = 255
                elif A[i][j] == 3:
                    B[i][j][0] = 255
                    B[i][j][1] = 255
                elif A[i][j] == 4:
                    B[i][j][0] = 255
                    B[i][j][2] = 255
                elif A[i][j] == 5:
                    B[i][j][1] = 255
                    B[i][j][2] = 255
                else:
                    print('Too many algorithms to represent color')
    if self.n_dims == 3:
        print('Too difficult to represent 3D gradient')
        return
    return B
  
  def interpolate_grid(self):
    self.interps = []
    for n in range (self.n_metrics):
        alg_interps = []
        for m in range (self.n_algs):
            if (self.n_dims == 1):
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size))), self.get_array_of_sim_metrics(n, m)))
            if (self.n_dims == 2):
                #print(self.x_vals)
                #print(self.y_vals)
                #print(self.get_array_of_sim_metrics(n, m))
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size)), np.reshape(self.y_vals, (self.grid_size))), self.get_array_of_sim_metrics(n, m)))
            if (self.n_dims == 3):
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size)), np.reshape(self.y_vals, (self.grid_size)), np.reshape(self.z_vals, (self.grid_size))), self.get_array_of_sim_metrics(n, m)))
        self.interps.append(alg_interps)
  
  def plot_surfaces(self, mode='save', filepath=''):
    self.interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        for m in range (self.n_algs):
            if self.n_dims == 1:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                plt_new = np.zeros(np.shape(xnew))
                for t in range (len(xnew)):
                    arr = np.array([xnew[t]]).reshape(self.n_dims)
                    plt_new[t] = self.interps[n][m](arr)
                fig = plt.figure()
                plt.plot(xnew, plt_new)
                plt.set_title(self.alg_names[m] + ' ' + self.metric_names[n] + ' Plot', fontsize=self.f_size)
                if (mode == 'save'):
                    plt.savefig(filepath + self.alg_names[m] + '_' + self.metric_names[n] + '_Plot.png')
                else:
                    plt.show()                       
            if self.n_dims == 2:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                X, Y = np.meshgrid(xnew, ynew)
                plt_new = np.zeros((len(xnew), len(ynew)))
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        arr = np.array([xnew[t], ynew[u]]).reshape(self.n_dims)
                        plt_new[t][u] = self.interps[n][m](arr)
                fig = plt.figure()
                ax = plt.axes(projection='3d')
                ax.plot_surface(X, Y, plt_new, cmap='viridis', edgecolor='none')
                ax.set_title(self.alg_names[m] + ' ' + self.metric_names[n] + ' Surface', fontsize=self.f_size)
                if (mode == 'save'):
                    plt.savefig(filepath + self.alg_names[m] + '_' + self.metric_names[n] + '_Surface.png')
                else:
                    plt.show()           
            if self.n_dims == 3:
                print('Surface in 3D too difficult to show')
    plt.close('all')
    
  def get_array_of_sim_metrics(self, metric_num, alg_num):
    if self.n_dims == 1:
        A = np.zeros((self.grid_size))
        for i in range (self.grid_size):
            A[i] = self.grid_similarities[i][metric_num][alg_num].val                                
    if self.n_dims == 2:
        A = np.zeros((self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                A[i][j] = self.grid_similarities[i][j][metric_num][alg_num].val
    if self.n_dims == 3:
        A = np.zeros((self.grid_size, self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for k in range (self.grid_size):
                    A[i][j][k] = self.grid_similarities[i][j][k][metric_num][alg_num].val
    return A
    
  def get_plots_at_point1(self, i, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    plt.plot(self.org_x, 'k')
    for n in range (self.n_algs):
        plt.plot(self.grid_deforms_x[i][n].traj, colors[n])
    plt.title('Deformations at grid point (' + str(i) + ')', fontsize=self.f_size)
    if (mode == 'save'):
        plt.savefig(filepath + 'Deformations at grid point (' + str(i) + ').png')
    else:
        plt.show()   
  
  def get_plots_at_point2(self, i, j, alg=-1, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    plt.plot(self.org_x, self.org_y, 'k')
    plt.plot(self.org_x[0], self.org_y[0], 'k+', markersize=14)
    plt.plot(self.org_x[self.traj_len - 1], self.org_y[self.traj_len - 1], 'ko', markersize=14)
    for n in range (self.n_algs):
        if (n == alg):
            plt.plot(self.grid_deforms_x[i][j][n].traj, self.grid_deforms_y[i][j][n].traj, colors[n], linewidth=5.0)
        else:
            plt.plot(self.grid_deforms_x[i][j][n].traj, self.grid_deforms_y[i][j][n].traj, colors[n])  
        plt.plot(self.grid_deforms_x[i][j][n].traj[0], self.grid_deforms_y[i][j][n].traj[0], colors[n] + '+', markersize=14)
    #plt.title('Deformations at grid point (' + str(i) + ', ' + str(j) + ')', fontsize=self.f_size)
    if (mode == 'save'):
        plt.savefig(filepath + 'Deformations at grid point (' + str(i) + ', ' + str(j) + ').png')
    else:
        plt.show()   
    
  def get_plots_at_point3(self, i, j, k, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(self.org_x, self.org_y, self.org_z, 'k')
    for n in range (self.n_algs):
        ax.plot(self.grid_deforms_x[i][j][k][n].traj, self.grid_deforms_y[i][j][k][n].traj, self.grid_deforms_z[i][j][k][n].traj, colors[n])
    plt.title('Deformations at grid point (' + str(i) + ', ' + str(j) + ', ' + str(k) + ')', fontsize=self.f_size)
    if (mode == 'save'):
        plt.savefig(filepath + 'Deformations at grid point (' + str(i) + ', ' + str(j) + ', ' + str(k) + ').png')
    else:
        plt.show()   
          
  def plot_sim(self, sim, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    self.interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        for m in range (self.n_algs):
            if self.n_dims == 1:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                fig = plt.figure()
                for t in range (len(xnew)):
                    arr = np.array([xnew[t]]).reshape(self.n_dims)
                    if self.interps[n][m](arr) > sim:
                        plt.plot(xnew[t], colors[m] + '.')
                plt.plot(self.org_x[0], 'k*')
                name = self.metric_names[n] + 'similarity of ' + str(sim) + ' Plot'
                print(name)
                #plt.set_title(name, fontsize=self.f_size)
                plt.xticks(self.x_vals)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()                       
            if self.n_dims == 2:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                fig = plt.figure()
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        #print('m: %d t: %d u: %d' % (m, t, u))
                        arr = np.array([xnew[t], ynew[u]]).reshape(self.n_dims)
                        if self.interps[n][m](arr) > sim:
                            plt.plot(xnew[t], ynew[u], colors[m] + '.')
                name = self.metric_names[n] + ' similarity of ' + str(sim) + ' for ' + self.alg_names[m] + ' Plot'
                print(name)
                #plt.title(name, fontsize=self.f_size)
                plt.plot(self.org_x[0], self.org_y[0], 'k*')
                plt.xticks(self.x_vals)
                plt.yticks(self.y_vals)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()           
            if self.n_dims == 3:
                print('Similarity plot in 3D too difficult to show')
    plt.close('all')
    
  def plot_sim_strongest(self, sim, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    self.interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        fig = plt.figure()
        if self.n_dims == 1:
            A = np.zeros((n_surf))
        if self.n_dims == 2:
            A = np.zeros((n_surf, n_surf))
        if self.n_dims == 3:
            A = np.zeros((n_surf, n_surf, n_surf))
        for m in range (self.n_algs):
            name = self.metric_names[n] + ' similarity of ' + str(sim) + 'for ' + self.alg_names[m] + ' Plot'
            print(name)
            if self.n_dims == 1:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                for t in range (len(xnew)):
                    arr = np.array([xnew[t]]).reshape(self.n_dims)
                    if self.interps[n][m](arr) > sim and self.interps[n][m](arr) > A[t]:
                        A[t] = self.interps[n][m](arr)
                        plt.plot(xnew[t], colors[m] + '.')
                plt.plot(self.org_x[0], 'k+', markersize=20)
                name = self.metric_names[n] + 'similarity of ' + str(sim) + ' Plot'
                #plt.set_title(name, fontsize=self.f_size)
                plt.xticks(self.x_vals)    
            if self.n_dims == 2:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        #print('m: %d t: %d u: %d' % (m, t, u))
                        arr = np.array([xnew[t], ynew[u]]).reshape(self.n_dims)
                        if self.interps[n][m](arr) > sim and self.interps[n][m](arr) > A[t][u]:
                            A[t][u] = self.interps[n][m](arr)
                            plt.plot(xnew[t], ynew[u], colors[m] + '.')
                #plt.title(name, fontsize=self.f_size)
                plt.plot(self.org_x[0], self.org_y[0], 'k+', markersize=20)
                plt.xticks(self.x_vals)
                plt.yticks(self.y_vals)
            if self.n_dims == 3:
                print('Similarity plot in 3D too difficult to show')
        if (mode == 'save'):
            plt.savefig(filepath + name + '.png')
        else:
            plt.show()           
    plt.close('all')
    

def main2():
    print('Starting')
    #filename = '../h5 files/hello2.h5'
    #hf = h5py.File(filename, 'r')
    #hello = hf.get('hello')
    #x_data = hello.get('resampled_x')
    #x_data = np.array(x_data)
    #y_data = hello.get('resampled_y')
    #y_data = np.array(y_data)
    #hf.close()
    shape_names = ['Circle', 'Infinity', 'Pi', 'Pyramids', 'Ribbon', 'Slanted_Square', 'Spiral', 'Straight_Ribbon', 'Three', 'Worm']
    for i in range (len(shape_names)):
            print(shape_names[i])
            filename = '../h5 files/' + shape_names[i] +'_drawing_demo.h5'
            hf = h5py.File(filename, 'r')
            demo = hf.get(shape_names[i])
            x_data = demo.get('x')
            x_data = np.array(x_data)
            y_data = demo.get('y')
            y_data = np.array(y_data)
            hf.close()
            #gsm(x_data, y_data, shape_names[i] + '_dmp_on', is_dmp_on=True)
            gsm(x_data, y_data, shape_names[i], is_dmp_on=False)

def my_hd2(x1, x2, y1, y2):
    org_traj = np.zeros((len(x1), 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1)
    org_traj[:, 1] = np.transpose(y1)
    comp_traj[:, 0] = np.transpose(x2)
    comp_traj[:, 1] = np.transpose(y2)
    return max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
    
def my_fd2(x1, x2, y1, y2):
    org_traj = np.zeros((len(x1), 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1)
    org_traj[:, 1] = np.transpose(y1)
    comp_traj[:, 0] = np.transpose(x2)
    comp_traj[:, 1] = np.transpose(y2)
    return similaritymeasures.frechet_dist(org_traj, comp_traj)

def main3():
    plt_fpath = '../pictures/lte_writing/test/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
    filename = '../h5 files/' + 'Worm' + '_drawing_demo.h5'
    hf = h5py.File(filename, 'r')
    demo = hf.get('Worm')
    x_data = demo.get('x')
    x_data = np.array(x_data)
    y_data = demo.get('y')
    y_data = np.array(y_data)
    hf.close()
    my_gsm = GSM()
    my_gsm.add_traj_dimension(x_data, 'x')
    my_gsm.add_traj_dimension(y_data, 'y')
    #print(x_data)
    #print(y_data)
    my_gsm.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_gsm.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_gsm.add_sim_metric(my_fd2, name='Frechet', is_disssim=True)
    my_gsm.add_sim_metric(my_hd2, name='Haussdorf', is_disssim=True)
    #my_gsm.add_traj_dimension(x_data, 'z')
    my_gsm.create_grid(3, [20, 20])
    my_gsm.deform_traj(plot=False)
    my_gsm.calc_metrics(d_sample=True)
    #my_gsm.plot_gradients(mode='show', filepath=plt_fpath)
    #my_gsm.plot_gradients(mode='save', filepath=plt_fpath)
    #my_gsm.plot_strongest_gradients(mode='show', filepath=plt_fpath)
    #my_gsm.plot_strongest_gradients(mode='save', filepath=plt_fpath)
    my_gsm.plot_surfaces(mode='show', filepath=plt_fpath)
    my_gsm.plot_surfaces(mode='save', filepath=plt_fpath)
    my_gsm.get_plots_at_point2(0, 1)
    my_gsm.get_plots_at_point2(2, 1)
    
def main4():
    #shape_names = ['Circle', 'Infinity', 'Pi', 'Pyramids', 'Ribbon', 'Slanted_Square', 'Spiral', 'Straight_Ribbon', 'Three', 'Worm']
    shape_names = ['Straight_Ribbon']
    for i in range (len(shape_names)):
            print(shape_names[i])
            filename = '../h5 files/' + shape_names[i] +'_drawing_demo.h5'
            hf = h5py.File(filename, 'r')
            demo = hf.get(shape_names[i])
            x_data = demo.get('x')
            x_data = np.array(x_data)
            y_data = demo.get('y')
            y_data = np.array(y_data)
            hf.close()
            plt_fpath = '../pictures/lte_writing/GSM/' + shape_names[i] + '/'
            try:
                os.makedirs(plt_fpath)
            except OSError:
                print ("Creation of the directory %s failed" % plt_fpath)
            else:
                print ("Successfully created the directory %s" % plt_fpath)
            my_gsm = GSM()
            my_gsm.add_traj_dimension(x_data, 'x')
            my_gsm.add_traj_dimension(y_data, 'y')
            my_gsm.add_deform_alg(ja.perform_ja_improved, 'JA')
            my_gsm.add_deform_alg(lte.perform_lte_improved, 'LTE')
            my_gsm.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
            my_gsm.add_sim_metric(my_fd2, name='Frechet', is_disssim=True)
            my_gsm.add_sim_metric(my_hd2, name='Haussdorf', is_disssim=True)
            #my_gsm.add_traj_dimension(x_data, 'z')
            my_gsm.create_grid(10, [20, 20])
            my_gsm.deform_traj(plot=False)
            my_gsm.calc_metrics(d_sample=True)
            my_gsm.save_results('straight_ribbon_deform_data.h5')
            #my_gsm.plot_gradients(mode='show', filepath=plt_fpath)
            #my_gsm.plot_gradients(mode='save', filepath=plt_fpath)
            #my_gsm.plot_strongest_gradients(mode='show', filepath=plt_fpath)
            #my_gsm.plot_strongest_gradients(mode='save', filepath=plt_fpath)
            #my_gsm.plot_surfaces(mode='show', filepath=plt_fpath)
            #my_gsm.plot_surfaces(mode='save', filepath=plt_fpath)
            #my_gsm.plot_sim(sim=0.9, mode='show')
            #my_gsm.get_plots_at_point2(0, 2, mode='show')
            #my_gsm.get_plots_at_point2(5, 0, mode='show')
            #my_gsm.get_plots_at_point2(6, 7, mode='show')
            
def main():
    #shape_names = ['Circle', 'Infinity', 'Pi', 'Pyramids', 'Ribbon', 'Slanted_Square', 'Spiral', 'Straight_Ribbon', 'Three', 'Worm']
    name = 'Straight_Ribbon'
    filename = '../h5 files/' + name +'_drawing_demo.h5'
    plt_fpath = 'project_data/'
    hf = h5py.File(filename, 'r')
    demo = hf.get(name)
    x_data = demo.get('x')
    x_data = np.array(x_data)
    #x_data = downsample_1d(x_data)
    y_data = demo.get('y')
    y_data = np.array(y_data)
    #y_data = downsample_1d(y_data)
    hf.close()
    fig = plt.figure()
    plt.plot(x_data, y_data, 'k')
    plt.plot(x_data[0], y_data[0], 'k+', markersize=14)
    plt.plot(x_data[len(x_data) - 1], y_data[len(y_data) - 1], 'ko', markersize=14)
    plt.show()
    #my_gsm = GSM()
    #my_gsm.add_traj_dimension(x_data, 'x')
    #my_gsm.add_traj_dimension(y_data, 'y')
    #my_gsm.add_deform_alg(ja.perform_ja_improved, 'JA')
    #my_gsm.add_deform_alg(lte.perform_lte_improved, 'LTE')
    #my_gsm.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    #my_gsm.add_sim_metric(my_fd2, name='Frechet', is_dissim=True)
    #my_gsm.add_sim_metric(my_hd2, name='Haussdorf', is_dissim=True)
    #my_gsm.add_traj_dimension(x_data, 'z')
    #my_gsm.create_grid(10, [20, 20])
    #my_gsm.create_grid(5, [20, 20])
    #my_gsm.deform_traj(plot=False)
    #my_gsm.calc_metrics(d_sample=True)
    #my_gsm.save_results('straight_ribbon_deform_data_full.h5')
    #my_gsm.read_from_h5('straight_ribbon_deform_data_full.h5')
    #my_gsm.plot_heatmap(mode='show')
    #my_gsm.plot_heatmap(mode='save', filepath=plt_fpath)
    #my_gsm.plot_gradients(mode='show')
    #my_gsm.plot_gradients(mode='save', filepath=plt_fpath)
    #my_gsm.plot_strongest_gradients(mode='show')
    #my_gsm.plot_strongest_gradients(mode='save', filepath=plt_fpath)
    #my_gsm.plot_surfaces(mode='show', filepath=plt_fpath)
    #my_gsm.plot_surfaces(mode='save', filepath=plt_fpath)
    #print(my_gsm.get_array_of_sim_metrics(0, 0))
    #print('input:')
    #a = float(input())
    #my_gsm.plot_sim_strongest(sim=0.72, mode='show')
    #my_gsm.get_plots_at_point2(0, 2, alg=0, mode='show')
    #my_gsm.get_plots_at_point2(5, 0, alg=2, mode='show')
    #my_gsm.get_plots_at_point2(6, 7, alg=1, mode='show')
     
if __name__ == '__main__':
  main()