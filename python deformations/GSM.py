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
    self.org_x = None
    self.org_y = None
    self.org_z = None
    self.algs = []
    self.alg_names = []
    self.metrics = []
    self.metric_names = []
    self.is_dissims = []
    
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
        if (self.org_x == None):
            self.org_x = traj
        elif (self.org_y == None):
            self.org_y = traj
        elif (self.org_z == None):
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
    
  def add_sim_metric(self, metric, name='', is_disssim=False):
    self.metrics.append(metric)
    self.metric_names.append(name)
    self.n_metrics = self.n_metrics + 1
    self.is_dissims.append(is_disssim)
    
  def create_grid(self, given_grid_size, dists, disp=False):
    self.grid_size = given_grid_size
    if (self.n_dims >= 1):
        center = point(self.org_x[0])
        grid_max_x = center.x + (dists[0] / 2)
        grid_min_x = center.x - (dists[0] / 2)
        self.x_vals = np.linspace(grid_min_x, grid_max_x, self.grid_size)
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
        self.grid_deforms_x = [[traj() for n in range (self.n_algs)] for i in range (self.grid_size)]
        self.grid_similarities = [[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)]
        for i in range (self.grid_size):
            self.grid[i].x = self.x_vals[i]
        if (disp == True):
            for i in range (self.grid_size):
                print('X: %f' % (self.grid[i].x))
    if (self.n_dims == 2):
        self.grid = [[point() for i in range (self.grid_size)] for j in range (self.grid_size)]
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
    
  def calc_metrics(self, d_sample=True, n_dsample=100):
    for n in range (self.n_metrics):
        A = np.zeros((self.grid_size, self.grid_size))
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
                        self.grid_similarities[i][j][n][m].val = my_map(self.grid_similarities[i][j][n][m].val, metric_min, metric_max, 1, 0)
                        A[i][j] = self.grid_similarities[i][j][n][m].val
            if self.n_dims == 3:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            self.grid_similarities[i][j][k][n][m].val = my_map(self.grid_similarities[i][j][k][n][m].val, metric_min, metric_max, 1, 0)
    else:
        for m in range (self.n_algs):
            if self.n_dims == 1:
                for i in range (self.grid_size):
                    self.grid_similarities[i][n][m].val = my_map(self.grid_similarities[i][n][m].val, metric_min, metric_max, 0, 1)
            if self.n_dims == 2:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        self.grid_similarities[i][j][n][m].val = my_map(self.grid_similarities[i][j][n][m].val, metric_min, metric_max, 0, 1)
                        A[i][j] = self.grid_similarities[i][j][n][m].val
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
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            if self.n_dims == 1:
                A = np.array((self.grid_size))
                for i in range (self.grid_size):
                    fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ')/x', data=self.grid_deforms_x[i][m].traj)
                    A[i] = self.grid_similarities[i][n][m].val
                fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/' + self.metric_names[n], data=A)
            if self.n_dims == 2:
                A = np.array((self.grid_size, self.grid_size))
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ')/x', data=self.grid_deforms_x[i][j][m].traj)
                        fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ')/y', data=self.grid_deforms_y[i][j][m].traj)
                        A[i][j] = self.grid_similarities[i][j][n][m].val
                fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/' + self.metric_names[n], data=A)
            if self.n_dims == 3:
                A = np.array((self.grid_size, self.grid_size, self.grid_size))
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/x', data=self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/y', data=self.grid_deforms_y[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/z', data=self.grid_deforms_z[i][j][k][m].traj)
                            A[i][j][k] = self.grid_similarities[i][j][k][n][m].val
                fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/' + self.metric_names[n], data=A)
    fp.close()
  
  def plot_gradients(self, mode='save', filepath=''):
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            if self.n_dims == 1:
                A = np.array((self.grid_size))
                for i in range (self.grid_size):
                    A[i] = self.grid_similarities[i][n][m].val
                if (mode == save):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')                                      
            if self.n_dims == 2:
                A = np.array((self.grid_size, self.grid_size))
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        A[i][j] = self.grid_similarities[i][j][n][m].val
                if (mode == save):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')   
            if self.n_dims == 3:
                A = np.array((self.grid_size, self.grid_size, self.grid_size))
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            A[i][j][k] = self.grid_similarities[i][j][k][n][m].val 
                if (mode == save):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')   
      
  def plot_surfaces(self, mode='save', filepath=''):
    return
        
        

        

#in-file testing
def gsm(x_data, y_data, name='', is_dmp_on=False, grid_size=5, grid_x_dist=-1.0, grid_y_dist=-1.0):
    plt_fpath = '../pictures/lte_writing/' + name + '/' + str(grid_size) + '_grid/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
    ## Optimize JA for trajectory ##
    print('Optimizing JA')
    lambda_x = optimize_ja.opt_lambda_traj_1d(x_data)
    lambda_y = optimize_ja.opt_lambda_traj_1d(y_data)
    ## Get deform grid ##
    print('Getting Deform Grid')
    #Constants--can be changed
    se_dist = get_start_end_dist(x_data, y_data)
    ttl_dist = get_total_dist(x_data, y_data)
    middle = ((se_dist + ttl_dist)**0.5)
    if (grid_x_dist < 0.0):
        grid_x_dist = middle
    if (grid_y_dist < 0.0):
        grid_y_dist = middle
    #Center should always be start of traj
    center = deform_hello_grid.point(x_data[0], y_data[0])
    grid = deform_hello_grid.create_grid(grid_size, grid_x_dist, grid_y_dist, center)
    ## deform for each point on grid ##
    print('Deforming Trajectory')
    grid_deforms_x = [[deform_hello_grid.deformation(x_data, grid[i][j].x, given_final=[], given_lambda=lambda_x, dmp_on=is_dmp_on) for i in range (grid_size)] for j in range (grid_size)]
    grid_deforms_y = [[deform_hello_grid.deformation(y_data, grid[i][j].y, given_final=[], given_lambda=lambda_y, dmp_on=is_dmp_on) for i in range (grid_size)] for j in range (grid_size)]
    ## get hd/fd for each deformation ##
    print('Getting hd/fd')
    #set up arrays
    starts_x = np.zeros((grid_size, grid_size))
    starts_y = np.zeros((grid_size, grid_size))
    fd_lte = np.zeros((grid_size, grid_size))
    hd_lte = np.zeros((grid_size, grid_size))
    fd_ja = np.zeros((grid_size, grid_size))
    hd_ja = np.zeros((grid_size, grid_size))
    if is_dmp_on:
        fd_dmp = np.zeros((grid_size, grid_size))
        hd_dmp = np.zeros((grid_size, grid_size))
    org_traj = np.zeros((len(x_data), 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x_data)
    org_traj[:, 1] = np.transpose(y_data)
    for i in range (grid_size):
        for j in range (grid_size):
            print('Starting')
            starts_x[i][j] = grid_deforms_x[i][j].lte[0]
            starts_y[i][j] = grid_deforms_y[i][j].lte[0]
            #lte hd/fd
            comp_traj[:, 0] = np.transpose(grid_deforms_x[i][j].lte)
            comp_traj[:, 1] = np.transpose(grid_deforms_y[i][j].lte)
            fd_lte[i][j] = similaritymeasures.frechet_dist(org_traj, comp_traj)
            hd_lte[i][j] = max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
            #ja hd/fd
            comp_traj[:, 0] = np.transpose(grid_deforms_x[i][j].ja)
            comp_traj[:, 1] = np.transpose(grid_deforms_y[i][j].ja)
            fd_ja[i][j] = similaritymeasures.frechet_dist(org_traj, comp_traj)
            hd_ja[i][j] = max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
            #dmp hd/fd
            if is_dmp_on:
                comp_traj[:, 0] = np.transpose(grid_deforms_x[i][j].dmp)
                comp_traj[:, 1] = np.transpose(grid_deforms_y[i][j].dmp)
                fd_dmp[i][j] = similaritymeasures.frechet_dist(org_traj, comp_traj)
                hd_dmp[i][j] = max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
    print(starts_x)
    print(starts_y)
    print(fd_lte)
    print(hd_lte)
    print(fd_ja)
    print(hd_ja)
    if is_dmp_on:
        print(fd_dmp)
        print(hd_dmp)
    ## normalize hd/fd ##
    print('Normalizing hd/fd')
    if is_dmp_on:
        #get maxes
        max_fd = max(np.amax(fd_dmp), np.amax(fd_dmp), np.amax(fd_dmp))
        max_hd = max(np.amax(hd_dmp), np.amax(hd_dmp), np.amax(hd_dmp))
        fd_dmp = np.ones((np.shape(fd_dmp))) - (fd_dmp / max_fd)
        hd_dmp = np.ones((np.shape(hd_dmp))) - (hd_dmp / max_hd)
    else:
        max_fd = max(np.amax(fd_lte), np.amax(fd_ja))
        max_hd = max(np.amax(hd_lte), np.amax(hd_ja))
    fd_lte = np.ones((np.shape(fd_lte))) - (fd_lte / max_fd)
    hd_lte = np.ones((np.shape(hd_lte))) - (hd_lte / max_hd)
    fd_ja = np.ones((np.shape(fd_ja))) - (fd_ja / max_fd)
    hd_ja = np.ones((np.shape(hd_ja))) - (hd_ja / max_hd)
    ## plot results ##
    print('Plotting Results')
    #plot deformations & store in h5
    print(name + '_grid' + str(grid_size) + '.h5')
    fp = h5py.File (name + '_grid' + str(grid_size) + '.h5', 'w')
    dset_name = name
    for i in range (grid_size):
        for j in range (grid_size):
            ax = plt.subplot2grid((grid_size, grid_size), (i, j))
            ax.plot(grid_deforms_x[i][j].traj, grid_deforms_y[i][j].traj, 'b')
            ax.plot(grid_deforms_x[i][j].lte, grid_deforms_y[i][j].lte, 'g')
            ax.plot(grid_deforms_x[i][j].ja, grid_deforms_y[i][j].ja, 'r')
            if is_dmp_on:
                ax.plot(grid_deforms_x[i][j].dmp, grid_deforms_y[i][j].dmp, 'm')
            fp.create_dataset(dset_name + '/original/(' + str(i) + ', ' + str(j) + ')/x', data=grid_deforms_x[i][j].traj)
            fp.create_dataset(dset_name + '/original/(' + str(i) + ', ' + str(j) + ')/y', data=grid_deforms_y[i][j].traj)
            fp.create_dataset(dset_name + '/lte/(' + str(i) + ', ' + str(j) + ')/x', data=grid_deforms_x[i][j].lte)
            fp.create_dataset(dset_name + '/lte/(' + str(i) + ', ' + str(j) + ')/y', data=grid_deforms_y[i][j].lte)
            fp.create_dataset(dset_name + '/ja/(' + str(i) + ', ' + str(j) + ')/x', data=grid_deforms_x[i][j].ja)
            fp.create_dataset(dset_name + '/ja/(' + str(i) + ', ' + str(j) + ')/y', data=grid_deforms_y[i][j].ja)
            if is_dmp_on:
                fp.create_dataset(dset_name + '/dmp/(' + str(i) + ', ' + str(j) + ')/x', data=grid_deforms_x[i][j].dmp)
                fp.create_dataset(dset_name + '/dmp/(' + str(i) + ', ' + str(j) + ')/y', data=grid_deforms_y[i][j].dmp)
    plt.xticks([])
    plt.yticks([])
    plt.savefig(plt_fpath + 'deforms.png')
    #store hd/fd data in h5
    fp.create_dataset(dset_name + '/lte/fd', data=fd_lte)
    fp.create_dataset(dset_name + '/lte/hd', data=hd_lte)
    fp.create_dataset(dset_name + '/ja/fd', data=fd_ja)
    fp.create_dataset(dset_name + '/ja/hd', data=hd_ja)
    if is_dmp_on:
        fp.create_dataset(dset_name + '/dmp/fd', data=fd_dmp)
        fp.create_dataset(dset_name + '/dmp/hd', data=hd_dmp)
    #gradient maps
    gradient_plotting.gradient_map(fd_lte, name + ' LTE Frechet Distance Gradient', fpath=plt_fpath)
    gradient_plotting.gradient_map(hd_lte, name + ' LTE Haussdorf Distance Gradient', fpath=plt_fpath)
    gradient_plotting.gradient_map(fd_ja, name + ' JA Frechet Distance Gradient', fpath=plt_fpath)
    gradient_plotting.gradient_map(hd_ja, name + ' JA Haussdorf Distance Gradient', fpath=plt_fpath)
    if is_dmp_on:
        gradient_plotting.gradient_map(fd_dmp, name + ' DMP Frechet Distance Gradient', fpath=plt_fpath)
        gradient_plotting.gradient_map(hd_dmp, name + ' DMP Haussdorf Distance Gradient', fpath=plt_fpath)
        gradient_plotting.rgb_gradient(fd_ja, fd_lte, fd_dmp, name=(name + ' Frechet Distance Compared Reproductions'), fpath=plt_fpath)
        gradient_plotting.rgb_gradient(hd_ja, hd_lte, hd_dmp, name=(name + ' Haussdorf Distance Compared Reproductions'), fpath=plt_fpath)
        gradient_plotting.strongest_gradient(fd_ja, fd_lte, fd_dmp, name=(name + ' Frechet Distance Best Reproductions'), fpath=plt_fpath)
        gradient_plotting.strongest_gradient(hd_ja, hd_lte, hd_dmp, name=(name + ' Haussdorf Distance Best Reproductions'), fpath=plt_fpath)
    else:
        gradient_plotting.rgb_gradient(fd_ja, fd_lte, np.zeros((np.shape(fd_lte))), name=(name + ' Frechet Distance Compared Reproductions'), fpath=plt_fpath)
        gradient_plotting.rgb_gradient(hd_ja, hd_lte, np.zeros((np.shape(fd_lte))), name=(name + ' Haussdorf Distance Compared Reproductions'), fpath=plt_fpath)
        gradient_plotting.strongest_gradient(fd_ja, fd_lte, np.zeros((np.shape(fd_lte))), name=(name + ' Frechet Distance Best Reproductions'), fpath=plt_fpath)
        gradient_plotting.strongest_gradient(hd_ja, hd_lte, np.zeros((np.shape(fd_lte))), name=(name + ' Haussdorf Distance Best Reproductions'), fpath=plt_fpath)
    #set up grid for 3d surfaces
    x_vals = starts_x[0, :]
    y_vals = starts_y[:, 0]
    xnew = np.linspace(x_vals[0], x_vals[grid_size - 1], 1000)
    ynew = np.linspace(y_vals[0], y_vals[grid_size - 1], 1000)
    X, Y = np.meshgrid(xnew, ynew)
    #interpolate functions & plot interpolations
    fd_lte_func = interp2d(x_vals, y_vals, fd_lte)
    fd_lte_plot = fd_lte_func(xnew, ynew)
    hd_lte_func = interp2d(x_vals, y_vals, hd_lte)
    hd_lte_plot = fd_lte_func(xnew, ynew)
    fd_ja_func = interp2d(x_vals, y_vals, fd_ja)
    fd_ja_plot = fd_ja_func(xnew, ynew)
    hd_ja_func = interp2d(x_vals, y_vals, hd_ja)
    hd_ja_plot = hd_ja_func(xnew, ynew)
    if is_dmp_on:
        fd_dmp_func = interp2d(x_vals, y_vals, fd_dmp)
        fd_dmp_plot = fd_dmp_func(xnew, ynew)
        hd_dmp_func = interp2d(x_vals, y_vals, hd_dmp)
        hd_dmp_plot = hd_dmp_func(xnew, ynew)
    #plot all three on a single plot?
    #have all seperate plots?
    #how to show which sirface is better at a single point?
    #different color maps
    #how do I show hd vs. fd?
    f_size = 32
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, fd_lte_plot,cmap='viridis', edgecolor='none')
    ax.set_title(name + ' LTE Frechet Distance', fontsize=f_size)
    #plt.show()
    plt.savefig(plt_fpath + name + ' LTE Frechet Distance Surface.png')
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, hd_lte_plot,cmap='viridis', edgecolor='none')
    ax.set_title(name + ' LTE Haussdorf Distance', fontsize=f_size)
    #plt.show()
    plt.savefig(plt_fpath + name + ' LTE Haussdorf Distance Surface.png')
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, fd_ja_plot,cmap='viridis', edgecolor='none')
    ax.set_title(name + ' JA Frechet Distance', fontsize=f_size)
    #plt.show()
    plt.savefig(plt_fpath + name + ' JA Frechet Distance Surface.png')
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, hd_ja_plot,cmap='viridis', edgecolor='none')
    ax.set_title(name + ' JA Haussdorf Distance', fontsize=f_size)
    #plt.show()
    plt.savefig(plt_fpath + name + ' JA Haussdorf Distance Surface.png')
    if is_dmp_on:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot_surface(X, Y, fd_dmp_plot,cmap='viridis', edgecolor='none')
        ax.set_title(name + ' DMP Frechet Distance', fontsize=f_size)
        #plt.show()
        plt.savefig(plt_fpath + name + ' DMP Frechet Distance Surface.png')
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot_surface(X, Y, hd_dmp_plot,cmap='viridis', edgecolor='none')
        ax.set_title(name + ' DMP Haussdorf Distance', fontsize=f_size)
        #plt.show()
        plt.savefig(plt_fpath + name + ' DMP Haussdorf Distance Surface.png')
    fp.close()
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
            gsm(x_data, y_data, shape_names[i], is_dmp_on=False)
            #gsm(x_data, y_data, shape_names[i] + '_dmp_on', is_dmp_on=True)
    #filename = '../h5 files/Circle_drawing_demo.h5'
    #hf = h5py.File(filename, 'r')
    #circ = hf.get('Circle')
    #x_data = circ.get('x')
    #x_data = np.array(x_data)
    #y_data = circ.get('y')
    #y_data = np.array(y_data)
    #hf.close()
    #gsm(x_data, y_data, 'Circle', is_dmp_on=False)

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
    #print('org_x0')
    #print(org_traj[0, 0])
    #print('org_y0')
    #print(org_traj[0, 1])
    #print('comp_x0')
    #print(comp_traj[0, 0])
    #print('comp_y0')
    #print(comp_traj[0, 1])
    #print('start: (%f, %f)' % (comp_traj[0][0], comp_traj[0][1]))
    return similaritymeasures.frechet_dist(org_traj, comp_traj)

def main():
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
    my_gsm.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_gsm.add_sim_metric(my_fd2, name='Haussdorf', is_disssim=True)
    #my_gsm.add_traj_dimension(x_data, 'z')
    my_gsm.create_grid(3, [10, 10])
    my_gsm.deform_traj(plot=False)
    my_gsm.calc_metrics(d_sample=True)
    my_gsm.plot_gradients(mode='show', filepath=plt_fpath)
    
    
if __name__ == '__main__':
  main()