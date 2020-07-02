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
from sklearn.neighbors import KDTree
import os
from scipy.interpolate import RegularGridInterpolator
import seaborn as sns; sns.set()
from sklearn.svm import SVC
import dtw

### TO-DO ###
#add weights to similarity metrics
#output reproduction at point selected
#better selection of point
#say which algorithm is best at point
#better way for val class
#make the gird an array and not a multidimensional list

#point class to define the grid
class point(object):
  #create the point object
  #if not all 3 arguments are given the point will not be 3 dimensional
  #arguments
  #given_x: double with x value of point
  #given_y(optional): double with y value of point
  #given_z(optional): double with z value of point
  #returns a point object with an (x, y, z) value
  def __init__(self, given_x=None, given_y=None, given_z=None):
    self.y = given_y
    self.x = given_x 
    self.z = given_z

#trajectory class used to store trajectories
class traj(object):
  #create the traj object
  #arguments
  #given_traj: row vector with 1 dimensional trajectory
  #returns a traj object that holds a 1D trajectory
  def __init__(self, given_traj=[]):
    self.traj = given_traj

#value class because I needed a list of values and I couldn't figure out a better way to do it
class val(object):
  #create the traj object
  #arguments
  #value: double holding a value
  #returns a val object with a value
  def __init__(self, value=0.0):
    self.val = value
    
#map function to map values from one min/max to another min/max
#arguments
#x: value to be mapped
#in_min: minimum value of the input
#in_max: maximum value of the input
#out_min: minimum value of the output
#out_max: maximum value of the output
#returns the input mapped to the range of the output
def my_map(x, in_min, in_max, out_min, out_max): #arduino's map function
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def my_get_max_from_column(A, col):
    my_max = None
    arr_shape = np.shape(A)
    ind = None
    for i in range (arr_shape[0]):
        if (my_max == None) or (A[i][col] > my_max):
            my_max = A[i][col]
            ind = i
    return my_max, ind

#function to downsample a 1 dimensional trajectory to n points (for multidimensional trajectories call the function multiple times
#arguments
#traj: 1 dimensional vector
#n (optional): the number of points to be in the downsampled trajectory. Default is 100.
#returns the trajectory downsampled to n points
def downsample_1d(traj, n=100):
    #print(np.shape(traj))
    npts = np.linspace(0, len(traj) - 1, n)
    out = np.zeros((n))
    #print(np.shape(traj))
    sz = np.shape(traj)
    if len(sz) > 1:
        if sz[0] < sz[1]:
            traj = np.transpose(traj)
    for i in range (n):
        out[i] = traj[int(npts[i])]
    #print(np.shape(out))
    return out

#simple function to get the euclidean distance between 2 2D points
#arguments
#x1: x value of the first point
#y1: y value of the first point
#x2: x value of the second point
#y2: y value of the second point
#returns euclidean distance between point 1 and point 2
def get_euclidian_dist(x1, y1, x2, y2):
    return ((x2-x1)**2 + (y2-y1)**2)**0.5

def get_euclidian_dist3(x1, y1, x2, y2, z1, z2):
    return ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5

#function to get the total travel distance of a 2D trajectory
#arguments
#x: x values of trajectory
#y: y values of trajectory
#returns total travel distance by summing contiguous euclidean distances of points
def get_total_dist(x, y):
    ttl = 0
    for i in range (len(x) - 2):
        ttl = ttl + get_euclidian_dist(x[i], y[i], x[i+1], y[i+1])
    return ttl
    
def get_total_dist3(x, y, z):
    ttl = 0
    for i in range (len(x) - 2):
        ttl = ttl + get_euclidian_dist3(x[i], y[i], x[i+1], y[i+1], z[i], z[i + 1])
    return ttl
    
#simple function to get the distance between the start and end of a 2D trajectory
#arguments
#x: x values of trajectory
#y: y values of trajectory
#returns total travel distance by getting euclidean distances of the inital and final points
def get_start_end_dist(x, y):
    return get_euclidian_dist(x[0], y[0], x[len(x) - 1], y[len(y) - 1])

#Meta-Lerning from Demonstration (MLfD) class
class mlfd(object):
  #initialize some elements of object
  #no arguments
  #returns mlfd object
  #mlfd = mlfd()
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
    self.metric_weights = []
    self.metric_weights_norm = []
    self.metric_types = []
    self.f_size = 32
    
  #add a peice of the trajectory to the object to store (added one dimension at a time, call multiple times to add multiple dimensions)
  #if no dimensions are specified, trajectories are added in order x->y->z (support for >3 trajectories not implemented here)
  #mlfd.add_traj_dimension(traj_1D)
  #arguments
  #self: calling object
  #traj: 1 dimensional trajectory
  #dim (optional): specifies if trajectory is for x, y, or z dimension
  #returns nothing
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
        
  #Add a deformation algorithm (LfD representation) to the object to store.
  #Ensure the function passed takes the arguments deform_1D = deform_algorithm(traj_1D, index, new_val)
  #mlfd.add_deform_alg(deform_algorithm, 'MY_DEFORM')
  #arguments
  #self: calling object
  #alg: reference to deformation algorithm function
  #name: string that denotes the name of the deformation algorithm
  #returns nothing
  def add_deform_alg(self, alg, name=''):
    self.algs.append(alg)
    self.alg_names.append(name)
    self.n_algs = self.n_algs + 1
    
  def use_default_deforms(self):
    self.add_deform_alg(ja.perform_ja_improved, 'FJA')
    self.add_deform_alg(lte.perform_lte_improved, 'LTE')
    
  #Add a similarity metric to the object to store
  #Ensure the metrix takes the arguments similarity_value = similarity_metric(x1, x2, y1, y2, z1, z2) where all arguments are vectors
  #mlfd.add_sim_metric(similarity_metric, 'MY_METRIC')
  #arguments
  #self: calling object
  #metric: reference to similarity metric function
  #name: string that denotes the name of the similarity metric
  #is_dissim (optional): if the algorithm is a measure of dissimilarity instead of similarity, set is_dissim to True
  #returns nothing
  #def add_sim_metric(self, metric, name='', weight=1.0, is_dissim=False):
  #  self.metrics.append(metric)
  #  self.metric_names.append(name)
  #  self.n_metrics = self.n_metrics + 1
  #  self.is_dissims.append(is_dissim)
  #  self.metric_weights.append(weight)
  #  self.metric_weights_norm = np.array(self.metric_weights) #I use this so weights_norm is the same shape while not pointing to the same reference
  #  sum = 0.0
  #  for n in range (self.n_metrics):
  #      sum = sum + self.metric_weights[n]
  #  for n in range (self.n_metrics):
  #      self.metric_weights_norm[n] = self.metric_weights[n] / sum
  #      if self.is_dissims[n]:
  #          self.metric_weights_norm[n] = -self.metric_weights_norm[n]
  #          #this is a temporary solution in order to combine dissimilarity metrics and similarity metrics
       
  def add_metric(self, metric, type='none', name='none', weight=1.0, is_dissim=False):
    if type in self.metric_types:
        print('Metric type already exists, given metric not added')
        return
    else:
        self.metrics.append(metric)
        self.metric_names.append(name)
        self.n_metrics = self.n_metrics + 1
        self.metric_weights.append(weight)
        self.metric_types.append(type)
        self.is_dissims.append(is_dissim)
        self.metric_weights_norm = np.array(self.metric_weights) #I use this so weights_norm is the same shape while not pointing to the same reference
        sum = 0.0
        for n in range (self.n_metrics):
            sum = sum + self.metric_weights[n]
        for n in range (self.n_metrics):
            self.metric_weights_norm[n] = self.metric_weights[n] / sum
    
  def use_default_metrics(self): #deprecated, replaced bt decide_metrics
    if self.n_dims == 1:
        print('1D metrics not implemented yet!')
    if self.n_dims == 2:
        self.add_metric(my_fd2, type='geometry', name='Frechet', weight=1.0, is_dissim=True)
    if self.n_dims == 3:
        self.add_metric(my_fd3, type='geometry', name='Frechet', weight=1.0, is_dissim=True)
    
  def decide_metrics(self):
    displace = self.get_demo_dist() / 10.0
    converge_alg = ja.perform_ja_improved
    preserve_alg = lte.perform_lte_improved
    if self.n_dims == 1:
        print('1D metrics not implemented yet!')
    if self.n_dims == 2:
        converge_metric = sum_of_dists
        preserve_metric = curvature_comparison
        temp_mlfd = mlfd()
        temp_mlfd.add_traj_dimension(self.org_x, 'x')
        temp_mlfd.add_traj_dimension(self.org_y, 'y')
        temp_mlfd.add_deform_alg(converge_alg, 'Converge')
        temp_mlfd.add_deform_alg(preserve_alg, 'Preserve')
        temp_mlfd.add_metric(similaritymeasures.frechet_dist, type='shape', name='Preserve', weight=1.0, is_dissim=True)
        temp_mlfd.reproduce_at_point(np.array([[self.org_x[0][0] + displace, self.org_y[0][0] + displace]]), plot=True)
        ext = False
        while not ext:
            ans = input('Which reproduction is preferable? Black is the given demonstration. Type "r" for red and "g" for green: ')
            if (ans == 'r'):
                ext = True
                self.add_metric(converge_metric, type='Converge', name='Converge', weight=1.0, is_dissim=True)
            elif (ans == 'g'):
                ext = True
                self.add_metric(preserve_metric, type='Preserve', name='Preserve', weight=1.0, is_dissim=True)
            else:
                print('Input not recognized. Please try again.')
    if self.n_dims == 3:
        converge_metric = sum_of_dists
        preserve_metric = curvature_comparison
        temp_mlfd = mlfd()
        temp_mlfd.add_traj_dimension(self.org_x, 'x')
        temp_mlfd.add_traj_dimension(self.org_y, 'y')
        temp_mlfd.add_traj_dimension(self.org_z, 'z')
        temp_mlfd.add_deform_alg(converge_alg, 'Converge')
        temp_mlfd.add_deform_alg(preserve_alg, 'Preserve')
        temp_mlfd.add_metric(similaritymeasures.frechet_dist, type='shape', name='Preserve', weight=1.0, is_dissim=True)
        temp_mlfd.reproduce_at_point(np.array([[self.org_x[0][0] + displace, self.org_y[0][0] + displace, self.org_z[0][0] + displace]]), plot=True)
        ext = False
        while not ext:
            ans = input('Which reproduction is preferable? Black is the given demonstration. Type "r" for red and "g" for green: ')
            if (ans == 'r'):
                ext = True
                self.add_metric(converge_metric, type='Converge', name='Converge', weight=1.0, is_dissim=True)
            elif (ans == 'g'):
                ext = True
                self.add_metric(preserve_metric, type='Preserve', name='Preserve', weight=1.0, is_dissim=True)
            else:
                print('Input not recognized. Please try again.')
    
  def get_demo_dist(self):
    if self.n_dims == 1:
        return self.org_x[-1] - self.org_x[0]
    if self.n_dims == 2:
        return get_total_dist(self.org_x, self.org_y)
    if self.n_dims == 3:
        return get_total_dist3(self.org_x, self.org_y, self.org_z)
    
  #Create the grid for which mlfd tests the generalization of the algorithms given against the metrics given
  #mlfd.create_grid(10, [20, 20, 20])
  #arguments
  #self: calling object
  #given_grid_size: number of points in the grid. For 1D, this is a line of N points. For 2D, this is an N x N square of points. For 3D, this is an N x N x N cube of points.
  #dists: string that denotes the name of the similarity metric
  #disp (optional): if the algorithm is a measure of dissimilarity instead of similarity, set is_dissim to True
  #returns nothing
  def create_grid_old(self, given_grid_size, dists, disp=False):
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
   
  def create_grid(self, given_grid_size=9, dists=None, disp=False):
    if (np.size(self.org_x) == 0):
        print('WARNING: No trajectories given')
    self.grid_size = given_grid_size
    if dists == None:
        K = 8.0
        dists = np.ones(self.n_dims) * (self.get_demo_dist() / K)
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
        self.grid_deforms_x = [[traj() for m in range (self.n_algs)] for i in range (self.grid_size)]
        self.grid_similarities = [[val() for m in range (self.n_algs)]  for i in range (self.grid_size)]
        self.grid_similarities_ind = [[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)]
        for i in range (self.grid_size):
            self.grid[i].x = self.x_vals[i]
        if (disp == True):
            for i in range (self.grid_size):
                print('X: %f' % (self.grid[i].x))
    if (self.n_dims == 2):
        self.grid = [[point() for i in range (self.grid_size)] for j in range (self.grid_size)]
        #self.grid_x, self.grid_y = np.meshgrid(x_vals, y_vals)
        self.grid_deforms_x = [[[traj() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        self.grid_deforms_y = [[[traj() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        self.grid_similarities = [[[val() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        self.grid_similarities_ind = [[[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)] for j in range (self.grid_size)]
        #plt.figure()
        #plt.axis('off')
        #plt.grid(b=None)
        #plt.plot(self.org_x[0], self.org_y[0], 'k*', markersize=30)
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                self.grid[j][i].x = self.x_vals[j]
                self.grid[j][i].y = self.y_vals[self.grid_size - 1 - i]
                #plt.plot(self.grid[j][i].x, self.grid[j][i].y, 'k.', markersize=20)
        if (disp == True):
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    print('X: %f, Y: %f' % (self.grid[i][j].x, self.grid[i][j].y))
        #plt.show()
    if (self.n_dims == 3):
        self.grid = [[[point() for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        #self.grid_x, self.grid_y, self.grid_z = np.meshgrid(x_vals, y_vals, z_vals)
        self.grid_deforms_x = [[[[traj() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        self.grid_deforms_y = [[[[traj() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)]
        self.grid_deforms_z = [[[[traj() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)] 
        self.grid_similarities = [[[[val() for m in range (self.n_algs)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)] 
        self.grid_similarities_ind = [[[[[val() for m in range (self.n_algs)] for n in range (self.n_metrics)] for i in range (self.grid_size)] for j in range (self.grid_size)] for k in range (self.grid_size)] 
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
    for m in range (self.n_algs):
        if self.n_dims == 1:
            for i in range (self.grid_size):
                self.grid_deforms_x[i][m].traj = self.algs[m](self.org_x, self.grid[i].x)
                if plot == True:
                    ax = plt.subplot2grid((self.grid_size), (i))
                    ax.plot(self.grid_deforms_x[i][m].traj, colors[m])
                    ax.plot(self.org_x, 'k')
                    plt.xticks([])
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    print('i: %d, j: %d, m: %d' % (i, j, m))
                    self.grid_deforms_x[i][j][m].traj = self.algs[m](self.org_x, self.grid[i][j].x)
                    self.grid_deforms_y[i][j][m].traj = self.algs[m](self.org_y, self.grid[i][j].y)
                    #print('start: (%f, %f)' % (self.grid_deforms_x[i][j][m].traj[0], self.grid_deforms_y[i][j][m].traj[0]))
                    if plot == True:
                        ax = plt.subplot2grid((self.grid_size, self.grid_size), (i, j))
                        ax.plot(self.grid_deforms_x[i][j][m].traj, self.grid_deforms_y[i][j][m].traj, colors[m])
                        ax.plot(self.org_x, self.org_y, 'k')
                        plt.xticks([])
                        plt.yticks([])
                        #plt.show()
        #    plt.show()
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        print('i: %d, j: %d, k: %d, m: %d' % (i, j, k, m))
                        #print(self.org_x)
                        self.grid_deforms_x[i][j][k][m].traj = self.algs[m](self.org_x, self.grid[i][j][k].x)
                        self.grid_deforms_y[i][j][k][m].traj = self.algs[m](self.org_y, self.grid[i][j][k].y)
                        self.grid_deforms_z[i][j][k][m].traj = self.algs[m](self.org_z, self.grid[i][j][k].z)
                        #print(self.grid_deforms_x[i][j][k][m].traj)
                        if plot == True:
                            print('Plotting a grid of 3D plots is too hard!')
    if plot == True:
        plt.show()
        plt.close('all')
    
  def get_deform_grid_2d(self, mode='show', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    plt.figure()
    for i in range (self.grid_size):
        for j in range (self.grid_size):
            plt.subplot(self.grid_size, self.grid_size, (self.grid_size * j + i + 1))
            for m in range (self.n_algs):
                #print(self.grid_deforms_x[i][j][m].traj)
                plt.plot(self.grid_deforms_x[i][j][m].traj, self.grid_deforms_y[i][j][m].traj, colors[m])
            plt.plot(self.org_x, self.org_y, 'k')
            plt.xticks([])
            plt.yticks([])
    if (mode == 'save'):
        plt.savefig(filepath + 'Grid_Deforms' + '.png')
    else:            
        plt.show() 
    plt.close('all')
    
    
  def calc_metrics_old(self, d_sample=True, n_dsample=100):
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
                    print(self._get_array_of_sim_metrics(n, m))
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
  
  def calc_metrics_deprecated(self, d_sample=True, n_dsample=100):
    #A = np.zeros((self.grid_size, self.grid_size))
    metric_max = None
    metric_min = None
    for m in range (self.n_algs):
        if d_sample == True:
            if self.n_dims == 1:
                for i in range (self.grid_size):
                    sim_val = 0
                    for n in range (self.n_metrics):
                        sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][m].traj, n_dsample)))
                    self.grid_similarities[i][m].val = sim_val
                    if (metric_max == None or self.grid_similarities[i][m].val > metric_max):
                        metric_max = self.grid_similarities[i][m].val
                    if (metric_min == None or self.grid_similarities[i][m].val < metric_min):
                        metric_min = self.grid_similarities[i][m].val
            if self.n_dims == 2:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        sim_val = 0
                        for n in range (self.n_metrics):
                            sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][j][m].traj, n_dsample), downsample_1d(self.org_y, n_dsample), downsample_1d(self.grid_deforms_y[i][j][m].traj, n_dsample)))
                        self.grid_similarities[i][j][m].val = sim_val
                        if (metric_max == None or self.grid_similarities[i][j][m].val > metric_max):
                            metric_max = self.grid_similarities[i][j][m].val
                        if (metric_min == None or self.grid_similarities[i][j][m].val < metric_min):
                            metric_min = self.grid_similarities[i][j][m].val
            if self.n_dims == 3:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            sim_val = 0
                            for n in range (self.n_metrics):
                                sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](downsample_1d(self.org_x, n_dsample), downsample_1d(self.grid_deforms_x[i][j][k][m].traj, n_dsample), downsample_1d(self.org_y, n_dsample), downsample_1d(self.grid_deforms_y[i][j][k][m].traj, n_dsample), downsample_1d(self.org_z, n_dsample), downsample_1d(self.grid_deforms_z[i][j][k][m].traj, n_dsample)))
                            self.grid_similarities[i][j][k][m].val = sim_val
                            if (metric_max == None or self.grid_similarities[i][j][k][m].val > metric_max):
                                metric_max = self.grid_similarities[i][j][k][m].val
                            if (metric_min == None or self.grid_similarities[i][j][k][m].val < metric_min):
                                metric_min = self.grid_similarities[i][j][k][m].val
        else:
            if self.n_dims == 1:
                for i in range (self.grid_size):
                    sim_val = 0
                    for n in range (self.n_metrics):
                        sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](self.org_x, self.grid_deforms_x[i][m].traj))
                    self.grid_similarities[i][m].val = sim_val
                    if (metric_max == None or self.grid_similarities[i][m].val > metric_max):
                        metric_max = self.grid_similarities[i][m].val
                    if (metric_min == None or self.grid_similarities[i][m].val < metric_min):
                        metric_min = self.grid_similarities[i][m].val
            if self.n_dims == 2:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        sim_val = 0
                        for n in range (self.n_metrics):
                            sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](self.org_x, self.grid_deforms_x[i][j][m].traj, self.org_y, self.grid_deforms_y[i][j][m].traj))
                        self.grid_similarities[i][j][m].val = sim_val
                        if (metric_max == None or self.grid_similarities[i][j][m].val > metric_max):
                            metric_max = self.grid_similarities[i][j][m].val
                        if (metric_min == None or self.grid_similarities[i][j][m].val < metric_min):
                            metric_min = self.grid_similarities[i][j][m].val
            if self.n_dims == 3:
                for i in range (self.grid_size):
                    for j in range (self.grid_size):
                        for k in range (self.grid_size):
                            sim_val = 0
                            for n in range (self.n_metrics):
                                sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](self.org_x, self.grid_deforms_x[i][j][k][m].traj, self.org_y, self.grid_deforms_y[i][j][k][m].traj, self.org_z, self.grid_deforms_z[i][j][k][m].traj))
                            self.grid_similarities[i][j][k][m].val = sim_val
                            if (metric_max == None or self.grid_similarities[i][j][k][m].val > metric_max):
                                metric_max = self.grid_similarities[i][j][k][m].val
                            if (metric_min == None or self.grid_similarities[i][j][k][m].val < metric_min):
                                metric_min = self.grid_similarities[i][j][k][m].val
    for m in range (self.n_algs):
        if self.n_dims == 1:
            for i in range (self.grid_size):
                self.grid_similarities[i][m].val = my_map(self.grid_similarities[i][m].val, metric_min, metric_max, 1, 0)
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    #print('n: %s m: %s i: %d j: %d org: %f' % (self.metric_names[n], self.alg_names[m], i, j, self.grid_similarities[i][j][n][m].val))
                    self.grid_similarities[i][j][m].val = my_map(self.grid_similarities[i][j][m].val, metric_min, metric_max, 1, 0)
                    #print('n: %s m: %s i: %d j: %d new: %f' % (self.metric_names[n], self.alg_names[m], i, j, self.grid_similarities[i][j][n][m].val))
                    #A[i][j] = self.grid_similarities[i][j][n][m].val
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        self.grid_similarities[i][j][k][m].val = my_map(self.grid_similarities[i][j][k][m].val, metric_min, metric_max, 1, 0)
    #    if (m == 3):
    #        print(self._get_array_of_sim_metrics(n, m))
    #print(A)
    #gradient_plotting.gradient_map_show(A, 'test', 0, 1)
  
  def calc_metrics(self, d_sample=True, n_dsample=100):
    for n in range (self.n_metrics):
        metric_max = None
        metric_min = None
        for m in range (self.n_algs):
            if d_sample == True:
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities_ind[i][n][m].val = self.metrics[n](np.hstack((np.reshape(downsample_1d(self.org_x, n_dsample), (n_dsample, 1)))), np.hstack((np.reshape(downsample_1d(self.grid_deforms_x[i][m].traj, n_dsample), (n_dsample, 1)))))
                        if (metric_max == None or self.grid_similarities_ind[i][n][m].val > metric_max):
                            metric_max = self.grid_similarities_ind[i][n][m].val
                        if (metric_min == None or self.grid_similarities_ind[i][n][m].val < metric_min):
                            metric_min = self.grid_similarities_ind[i][n][m].val
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            print('m: %d, i: %d, j: %d' % (m, i, j))
                            self.grid_similarities_ind[i][j][n][m].val = self.metrics[n](np.hstack((np.reshape(downsample_1d(self.org_x, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.org_y, n_dsample), (n_dsample, 1)))), np.hstack((np.reshape(downsample_1d(self.grid_deforms_x[i][j][m].traj, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.grid_deforms_y[i][j][m].traj, n_dsample), (n_dsample, 1)))))
                            if (metric_max == None or self.grid_similarities_ind[i][j][n][m].val > metric_max):
                                metric_max = self.grid_similarities_ind[i][j][n][m].val
                            if (metric_min == None or self.grid_similarities_ind[i][j][n][m].val < metric_min):
                                metric_min = self.grid_similarities_ind[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities_ind[i][j][k][n][m].val = self.metrics[n](np.hstack((np.reshape(downsample_1d(self.org_x, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.org_y, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.org_z, n_dsample), (n_dsample, 1)))), np.hstack((np.reshape(downsample_1d(self.grid_deforms_x[i][j][k][m].traj, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.grid_deforms_y[i][j][k][m].traj, n_dsample), (n_dsample, 1)), np.reshape(downsample_1d(self.grid_deforms_z[i][j][k][m].traj, n_dsample), (n_dsample, 1)))))
                                if (metric_max == None or self.grid_similarities_ind[i][j][k][n][m].val > metric_max):
                                    metric_max = self.grid_similarities_ind[i][j][k][n][m].val
                                if (metric_min == None or self.grid_similarities_ind[i][j][k][n][m].val < metric_min):
                                    metric_min = self.grid_similarities_ind[i][j][k][n][m].val
            else:
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        self.grid_similarities_ind[i][n][m].val = self.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)))), np.hstack((np.reshape(self.grid_deforms_x[i][m].traj, (self.traj_len, 1)))))
                        if (metric_max == None or self.grid_similarities_ind[i][n][m].val > metric_max):
                            metric_max = self.grid_similarities_ind[i][n][m].val
                        if (metric_min == None or self.grid_similarities_ind[i][n][m].val < metric_min):
                            metric_min = self.grid_similarities_ind[i][n][m].val
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            self.grid_similarities_ind[i][j][n][m].val = self.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)), np.reshape(self.org_y, (self.traj_len, 1)))), np.hstack((np.reshape(self.grid_deforms_x[i][j][m].traj, (self.traj_len, 1)), np.reshape(self.grid_deforms_y[i][j][m].traj, (self.traj_len, 1)))))
                            if (metric_max == None or self.grid_similarities_ind[i][j][n][m].val > metric_max):
                                metric_max = self.grid_similarities_ind[i][j][n][m].val
                            if (metric_min == None or self.grid_similarities_ind[i][j][n][m].val < metric_min):
                                metric_min = self.grid_similarities_ind[i][j][n][m].val
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                self.grid_similarities_ind[i][j][k][n][m].val = self.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)), np.reshape(self.org_y, (self.traj_len, 1)), np.reshape(self.org_z, (self.traj_len, 1)))), np.hstack((np.reshape(self.grid_deforms_x[i][j][k][m].traj, (self.traj_len, 1)), np.reshape(self.grid_deforms_y[i][j][k][m].traj, (self.traj_len, 1)), np.reshape(self.grid_deforms_z[i][j][k][m].traj, (self.traj_len, 1)))))
                                if (metric_max == None or self.grid_similarities_ind[i][j][k][n][m].val > metric_max):
                                    metric_max = self.grid_similarities_ind[i][j][k][n][m].val
                                if (metric_min == None or self.grid_similarities_ind[i][j][k][n][m].val < metric_min):
                                    metric_min = self.grid_similarities_ind[i][j][k][n][m].val
                                print(self.grid_similarities_ind[i][j][k][n][m].val)
        if (self.is_dissims[n] == True):
            for m in range (self.n_algs):
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        if math.isnan(self.grid_similarities_ind[i][n][m].val):
                            self.grid_similarities_ind[i][n][m].val = 0.
                        self.grid_similarities_ind[i][n][m].val = my_map(self.grid_similarities_ind[i][n][m].val, metric_min, metric_max, 1, 0)
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            if math.isnan(self.grid_similarities_ind[i][j][n][m].val):
                                self.grid_similarities_ind[i][j][n][m].val = 0.
                            self.grid_similarities_ind[i][j][n][m].val = my_map(self.grid_similarities_ind[i][j][n][m].val, metric_min, metric_max, 1, 0)
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                if math.isnan(self.grid_similarities_ind[i][j][k][n][m].val):
                                    self.grid_similarities_ind[i][j][k][n][m].val = 0.
                                self.grid_similarities_ind[i][j][k][n][m].val = my_map(self.grid_similarities_ind[i][j][k][n][m].val, metric_min, metric_max, 1, 0)
                if (m == 3):
                    print(self._get_array_of_sim_metrics(n, m))
        else:
            for m in range (self.n_algs):
                if self.n_dims == 1:
                    for i in range (self.grid_size):
                        if math.isnan(self.grid_similarities_ind[i][n][m].val):
                            self.grid_similarities_ind[i][n][m].val = 1.
                        self.grid_similarities_ind[i][n][m].val = my_map(self.grid_similarities_ind[i][n][m].val, metric_min, metric_max, 0, 1)
                if self.n_dims == 2:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            if math.isnan(self.grid_similarities_ind[i][j][n][m].val):
                                self.grid_similarities_ind[i][j][n][m].val = 1.
                            self.grid_similarities_ind[i][j][n][m].val = my_map(self.grid_similarities_ind[i][j][n][m].val, metric_min, metric_max, 0, 1)
                if self.n_dims == 3:
                    for i in range (self.grid_size):
                        for j in range (self.grid_size):
                            for k in range (self.grid_size):
                                if math.isnan(self.grid_similarities_ind[i][j][k][n][m].val):
                                    self.grid_similarities_ind[i][j][k][n][m].val = 1.
                                self.grid_similarities_ind[i][j][k][n][m].val = my_map(self.grid_similarities_ind[i][j][k][n][m].val, metric_min, metric_max, 0, 1)
    for m in range (self.n_algs):
        if self.n_dims == 1:
            for i in range (self.grid_size):
                sim_val = 0
                for n in range (self.n_metrics):
                    sim_val = sim_val + (self.metric_weights_norm[n] * self.grid_similarities_ind[i][n][m].val)
                self.grid_similarities[i][m].val = sim_val
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    sim_val = 0
                    for n in range (self.n_metrics):
                        print('m: %d, i: %d, j: %d, n: %d, sim: %f, weight: %f' % (m, i, j, n, self.grid_similarities_ind[i][j][n][m].val, self.metric_weights_norm[n]))
                        sim_val = sim_val + (self.metric_weights_norm[n] * self.grid_similarities_ind[i][j][n][m].val)
                    self.grid_similarities[i][j][m].val = sim_val
                    print(self.grid_similarities[i][j][m].val)
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        sim_val = 0
                        for n in range (self.n_metrics):
                            sim_val = sim_val + (self.metric_weights_norm[n] * self.grid_similarities_ind[i][j][k][n][m].val)
                        self.grid_similarities[i][j][k][m].val = sim_val
  
  def default_process(self):
    self.use_default_deforms()
    self.decide_metrics()
    self.create_grid()
    self.deform_traj(plot=False)
    self.calc_metrics(d_sample=True)
  
  def save_results_old(self, filename='unspecified.h5'):
    fp = h5py.File(filename, 'w')
    dset_name = 'mlfd'
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
                            #print(self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/x', data=self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/y', data=self.grid_deforms_y[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/z', data=self.grid_deforms_z[i][j][k][m].traj)
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            A = self._get_array_of_sim_metrics_old(n, m)
            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/' + self.metric_names[n], data=A)
    fp.close()
    
  def save_results(self, filename='unspecified.h5'):
    fp = h5py.File(filename, 'w')
    dset_name = 'mlfd'
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
                            #print(self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/x', data=self.grid_deforms_x[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/y', data=self.grid_deforms_y[i][j][k][m].traj)
                            fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')/z', data=self.grid_deforms_z[i][j][k][m].traj)
    for m in range (self.n_algs):
        A = self._get_array_of_sim_metrics(m)
        print(A)
        fp.create_dataset(dset_name + '/' + self.alg_names[m] + '/weighted_similarities', data=A)
    fp.close()
    
  def read_from_h5_old(self, filename):
    fp = h5py.File(filename, 'r')
    dset_name = 'mlfd'
    dset = fp.get(dset_name)
    gs = dset.get('grid_sz')
    gs = np.array(gs)
    self.create_grid_old(gs, [0, 0, 0])
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
                        print(deform_name)
                        deform = alg.get(deform_name)
                        x = deform.get('x')
                        self.grid_deforms_x[i][j][k][m].traj = np.array(x)
                        y = deform.get('y')
                        self.grid_deforms_y[i][j][k][m].traj = np.array(y)
                        z = deform.get('z')
                        self.grid_deforms_z[i][j][k][m].traj = np.array(z)
                        #print(self.grid_deforms_z[i][j][k][m].traj)
                        if (j == 0 and k == 0):
                            self.x_vals[i] = self.grid_deforms_x[i][j][k][m].traj[0]
                        if (i == 0 and k == 0):
                            self.y_vals[self.grid_size - 1 - j] = self.grid_deforms_y[i][j][k][m].traj[0]
                        if (i == 0 and j == 0):
                            self.z_vals[k] = self.grid_deforms_z[i][j][k][m].traj[0]
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

  def read_from_h5(self, filename):
    fp = h5py.File(filename, 'r')
    dset_name = 'mlfd'
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
                        if len(np.shape(self.grid_deforms_x[i][j][m].traj)) > 1:
                            self.x_vals[i] = self.grid_deforms_x[i][j][m].traj[0][0]
                        else:
                            self.x_vals[i] = self.grid_deforms_x[i][j][m].traj[0]
                    if (i == 0):
                        if len(np.shape(self.grid_deforms_y[i][j][m].traj)) > 1:
                            self.y_vals[self.grid_size - 1 - j] = self.grid_deforms_y[i][j][m].traj[0][0]
                        else:
                            self.y_vals[self.grid_size - 1 - j] = self.grid_deforms_y[i][j][m].traj[0]
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        deform_name = '(' + str(i) + ', ' + str(j) + ', ' + str(k) + ')'
                        print(deform_name)
                        deform = alg.get(deform_name)
                        x = deform.get('x')
                        self.grid_deforms_x[i][j][k][m].traj = np.array(x)
                        y = deform.get('y')
                        self.grid_deforms_y[i][j][k][m].traj = np.array(y)
                        z = deform.get('z')
                        self.grid_deforms_z[i][j][k][m].traj = np.array(z)
                        #print(self.grid_deforms_z[i][j][k][m].traj)
                        if (j == 0 and k == 0):
                            self.x_vals[i] = self.grid_deforms_x[i][j][k][m].traj[0]
                        if (i == 0 and k == 0):
                            self.y_vals[self.grid_size - 1 - j] = self.grid_deforms_y[i][j][k][m].traj[0]
                        if (i == 0 and j == 0):
                            self.z_vals[k] = self.grid_deforms_z[i][j][k][m].traj[0]
        for n in range (self.n_metrics):
            metric_name = self.metric_names[n]
        metric = alg.get('weighted_similarities')
        metric = np.array(metric)
        if self.n_dims == 1:
            for i in range (self.grid_size):
                self.grid_similarities[i][m].val = metric[i]
        if self.n_dims == 2:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    print(metric[i][j])
                    self.grid_similarities[i][j][m].val = metric[i][j]
        if self.n_dims == 3:
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        self.grid_similarities[i][j][k][m].val = metric[i][j][k]
                        print(self.grid_similarities[i][j][k][m].val)
    fp.close()
  
  def plot_gradients(self, mode='save', filepath=''):
    for m in range (self.n_algs):
        for n in range (self.n_metrics):
            if self.n_dims == 1:
                A = self._get_array_of_sim_metrics(n, m)
                if (mode == 'save'):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')                                      
            if self.n_dims == 2:
                A = self._get_array_of_sim_metrics(n, m)
                if (mode == 'save'):
                    gradient_plotting.gradient_map(A, self.alg_names[m] + self.metric_names[n] + 'Gradient', filepath)
                else:
                    gradient_plotting.gradient_map_show(A, self.alg_names[m] + self.metric_names[n] + 'Gradient')   
            if self.n_dims == 3:
                A = self._get_array_of_sim_metrics(n, m)
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
            A = self._get_array_of_sim_metrics(m)
            ax = sns.heatmap(np.transpose(A), annot=False)
            plt.xticks([])
            plt.yticks([])
            if (mode == 'save'):
                plt.savefig(filepath + name + '.png')
            else:            
                plt.show() 
            plt.close('all')

  def plot_strongest_gradients(self, mode='save', filepath=''):
    if self.n_dims == 1:
        A = np.zeros((self.grid_size))
        for i in range (self.grid_size):
            max_s = None
            for m in range (self.n_algs):
                if (max_s == None or max_s < self.grid_similarities[i][n][m].val):
                    max_s = self.grid_similarities[i][n][m].val
                    max_m = m
            A[i] = max_m
        B = self._convert_num_to_rgb(A)
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
                    if (max_s == None or max_s < self.grid_similarities[i][j][m].val):
                        max_s = self.grid_similarities[i][j][m].val
                        max_m = m
                A[i][j] = max_m
        B = self._convert_num_to_rgb(A)
        im = plt.imshow(B, vmin=0, vmax=1)
        plt.xticks([])
        plt.yticks([])
        #plt.title(self.metric_names[0] + 'Comparison', fontsize=self.f_size)
        if (mode == 'save'):
            plt.savefig(filepath + self.metric_names[0] + 'Comparison' + '.png')
        else:
            plt.show()           
    if self.n_dims == 3:
        print('Gradient in 3D too difficult to show')
    plt.close('all')
    
  def plot_strongest_gradients_thresholded(self, mode='save', filepath='', threshold=0.1):
    if self.n_dims == 1:
        print('Coming Soon!')
    #    A = np.zeros((self.grid_size))
    #    for i in range (self.grid_size):
    #        max_s = None
    #        for m in range (self.n_algs):
    #            if (max_s == None or max_s < self.grid_similarities[i][n][m].val):
    #                max_s = self.grid_similarities[i][n][m].val
    #                max_m = m
    #        A[i] = max_m
    #    B = self._convert_num_to_rgb(A)
    #    im = plt.imshow(np.transpose(B), vmin=0, vmax=1)
    #    plt.xticks([])
    #    plt.title(self.metric_names[n] + 'Comparison', fontsize=self.f_size)
    #    if (mode == 'save'):
    #        plt.savefig(filepath + self.metric_names[n] + 'Comparison' + '.png')
    #    else:
    #        plt.show()                                     
    if self.n_dims == 2:
        A = np.zeros((self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                max_s = None
                sims = np.zeros((self.n_algs))
                for m in range (self.n_algs):
                    sims[m] = self.grid_similarities[i][j][m].val
                if (max(sims) - min(sims) < threshold):
                    A[i][j] = -1
                else:
                    for m in range (self.n_algs):
                        if (max_s == None or max_s < self.grid_similarities[i][j][m].val):
                            max_s = self.grid_similarities[i][j][m].val
                            max_m = m
                    A[i][j] = max_m
        B = self._convert_num_to_rgb(A)
        im = plt.imshow(B, vmin=0, vmax=1)
        plt.xticks([])
        plt.yticks([])
        #plt.title(self.metric_names[0] + 'Comparison', fontsize=self.f_size)
        if (mode == 'save'):
            plt.savefig(filepath + self.metric_names[0] + 'Comparison' + '.png')
        else:
            plt.show()           
    if self.n_dims == 3:
        print('Gradient in 3D too difficult to show')
    plt.close('all')
    
  def _get_strongest_repro_old(self, threshold=0.0):
    for n in range (self.n_metrics):
        if self.n_dims == 1:
            A = np.zeros((self.grid_size, self.n_metrics))
            for i in range (self.grid_size):
                max_s = threshold
                max_m = -1
                for m in range (self.n_algs):
                    if (max_s < self.grid_similarities[i][n][m].val):
                        max_s = self.grid_similarities[i][n][m].val
                        max_m = m
                A[i][n] = max_m                                
        if self.n_dims == 2:
            A = np.zeros((self.grid_size, self.grid_size, self.n_metrics))
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    max_s = threshold
                    max_m = -1
                    for m in range (self.n_algs):
                        if (max_s < self.grid_similarities[i][j][n][m].val):
                            max_s = self.grid_similarities[i][j][n][m].val
                            max_m = m
                    A[i][j][n] = max_m
        if self.n_dims == 3:
            A = np.zeros((self.grid_size, self.grid_size, self.grid_size, self.n_metrics))
            for i in range (self.grid_size):
                for j in range (self.grid_size):
                    for k in range (self.grid_size):
                        max_s = threshold
                        max_m = -1
                        for m in range (self.n_algs):
                            if (max_s < self.grid_similarities[i][j][k][n][m].val):
                                max_s = self.grid_similarities[i][j][k][n][m].val
                                max_m = m
                        A[i][j][k][n] = max_m
                    print(max_m)
                    #print(self.alg_names[max_m])
    return A
    
  def _get_strongest_repro(self, threshold=0.0):
    #for m in range (self.n_algs):
    #    if self.n_dims == 1:
    #        A = np.zeros((self.grid_size))
    #        for i in range (self.grid_size):
    #            max_s = threshold
    #            max_m = -1
    #            if (max_s < self.grid_similarities[i][m].val):
    #                max_s = self.grid_similarities[i][m].val
    #                max_m = m
    #            A[i] = max_m                                
    #    if self.n_dims == 2:
    #        A = np.zeros((self.grid_size, self.grid_size))
    #        for i in range (self.grid_size):
    #            for j in range (self.grid_size):
    #                max_s = threshold
    #                max_m = -1
    #                if (max_s < self.grid_similarities[i][j][m].val):
    #                    max_s = self.grid_similarities[i][j][m].val
    #                    max_m = m
    #                A[i][j] = max_m
    #    if self.n_dims == 3:
    #        A = np.zeros((self.grid_size, self.grid_size, self.grid_size))
    #        for i in range (self.grid_size):
    #            for j in range (self.grid_size):
    #                for k in range (self.grid_size):
    #                    max_s = threshold
    #                    max_m = -1
    #                    if (max_s < self.grid_similarities[i][j][k][m].val):
    #                        max_s = self.grid_similarities[i][j][k][m].val
    #                        max_m = m
    #                    A[i][j][k] = max_m
    #                #print(max_m)
    #                #print(self.alg_names[max_m])
    if self.n_dims == 1:
        A = np.zeros((self.grid_size))
        for i in range (self.grid_size):
            max_m = -1
            max_s = threshold
            for m in range (self.n_algs):
                if (self.grid_similarities[i][m].val > max_s):
                    max_m = m
                    max_s = self.grid_similarities[i][m].val
            A[i] = max_m
    if self.n_dims == 2:
        A = np.zeros((self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                max_m = -1
                max_s = threshold
                for m in range (self.n_algs):
                    print(self.grid_similarities[i][j][m].val)
                    if (self.grid_similarities[i][j][m].val > max_s):
                        max_m = m
                        max_s = self.grid_similarities[i][j][m].val
                A[i][j] = max_m
                print('\n')
    if self.n_dims == 3:
        A = np.zeros((self.grid_size, self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for k in range (self.grid_size):
                    max_m = -1
                    max_s = threshold
                    for m in range (self.n_algs):
                        print(self.grid_similarities[i][j][k][m].val)
                        if (self.grid_similarities[i][j][k][m].val > max_s):
                            max_m = m
                            max_s = self.grid_similarities[i][j][k][m].val
                    A[i][j][k] = max_m
    return A
    
  def set_up_classifier(self, threshold=0.0):
    print('NOTE: This function assumes similarities have been normalized, meaning that for every point there is 1 similarity value!')
    X = np.zeros((self.grid_size**self.n_dims, self.n_dims))
    Y = np.zeros((self.grid_size**self.n_dims))
    print(np.shape(X))
    A = self._get_strongest_repro(threshold)
    if self.n_dims == 1:    
        for i in range (self.grid_size):
            X[i][0] = self.grid_deforms_x[i][0].traj[0]
            Y[i] = A[i]
    if self.n_dims == 2:
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                #essentially convert to base grid_size
                X[(j * self.grid_size) + (i)][0] = self.grid_deforms_x[i][j][0].traj[0]
                X[(j * self.grid_size) + (i)][1] = self.grid_deforms_y[i][j][0].traj[0]
                Y[(j * self.grid_size) + (i)] = A[i][j]
    if self.n_dims == 3:
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for k in range (self.grid_size):
                    X[(k * self.grid_size**2) + (j * self.grid_size) + (i)][0] = self.grid_deforms_x[i][j][k][0].traj[0]
                    X[(k * self.grid_size**2) + (j * self.grid_size) + (i)][1] = self.grid_deforms_y[i][j][k][0].traj[0]
                    X[(k * self.grid_size**2) + (j * self.grid_size) + (i)][2] = self.grid_deforms_z[i][j][k][0].traj[0]
                    Y[(k * self.grid_size**2) + (j * self.grid_size) + (i)] = A[i][j][k]
                    print(X[(k * self.grid_size**2) + (j * self.grid_size) + (i)])
                    print(Y[(k * self.grid_size**2) + (j * self.grid_size) + (i)])
    self.tree = KDTree(X)
    self.Y = Y
    self.X = X
    self.clf = SVC()
    print(X)
    print(Y)
    self.clf.fit(X, Y)
  
  def query_knn(self, coords, new_k=1, plot=False):
    dist, ind = self.tree.query(coords, k=new_k)
    res = np.zeros((self.n_algs + 1, 2))
    res[0][0] = -1
    for m in range (self.n_algs):
        res[m + 1][0] = m
    for p in range (new_k):
        for r in range (self.n_algs + 1):
            if (res[r][0] == self.Y[ind[0][p]]):
                res[r][1] = res[r][1] + 1
    max_n, max_alg_ind = my_get_max_from_column(res, 1)
    if (max_alg_ind == 0):
        print('No reproduction to fit threshold at given point')
    else:
        print('Optimal reproduction from: %s' % (self.alg_names[int(res[max_alg_ind][0])]))
    self.reproduce_at_point(coords, plot=plot)
            
  def query_svm(self, coords, plot=False):
    print(self.X)
    print(coords)
    if self.clf.predict(coords) < 0:
        print('No reproduction to fit threshold at given point')
    else:
        print('Optimal Reproduction at point ' + np.array2string(coords) + ' from ' + self.alg_names[int(self.clf.predict(coords))])
    self.reproduce_at_point(coords, plot=plot)
  
  def reproduce_at_point(self, coords, plot=False):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    if self.n_dims == 1:
        self.current_deforms_x = [traj() for m in range (self.n_algs)]
    if self.n_dims == 2:
        self.current_deforms_x = [traj() for m in range (self.n_algs)]
        self.current_deforms_y = [traj() for m in range (self.n_algs)]
    if self.n_dims == 3:
        self.current_deforms_x = [traj() for m in range (self.n_algs)]
        self.current_deforms_y = [traj() for m in range (self.n_algs)]
        self.current_deforms_z = [traj() for m in range (self.n_algs)]
    if plot == True:
        fig = plt.figure()
        if self.n_dims == 2:
                plt.plot(self.org_x, self.org_y, 'k', linewidth=6.0)
        if self.n_dims == 3:
            ax = fig.add_subplot(111, projection='3d')
    if self.n_dims == 1:
        for m in range (self.n_algs):
            self.current_deforms_x[m].traj = self.algs[m](self.org_x, coords[0][0])
            if plot == True:
                plt.plot(self.current_deforms_x[m].traj, colors[m])
                plt.plot(self.org_x, 'k', linewidth=6.0)
                plt.xticks([])
    if self.n_dims == 2:
        for m in range (self.n_algs):
            self.current_deforms_x[m].traj = self.algs[m](self.org_x, coords[0][0])
            self.current_deforms_y[m].traj = self.algs[m](self.org_y, coords[0][1])
            if plot == True:
                #fig.hold(True)
                #print(self.current_deforms_x[m].traj)
                #print(self.current_deforms_y[m].traj)
                plt.plot(self.current_deforms_x[m].traj, self.current_deforms_y[m].traj, colors[m])
                #plt.plot(self.org_x, self.org_y, 'k')
                #plt.show()
    if self.n_dims == 3:
        for m in range (self.n_algs):
            #print(coords[0][0])
            self.current_deforms_x[m].traj = self.algs[m](self.org_x, coords[0][0])
            #print('received value')
            #print(self.current_deforms_x[m].traj)
            #input('Press "Enter" to continue')
            self.current_deforms_y[m].traj = self.algs[m](self.org_y, coords[0][1])
            self.current_deforms_z[m].traj = self.algs[m](self.org_z, coords[0][2])
            if plot == True:
                ax.plot(self.org_x, self.org_y, self.org_z, 'k', linewidth=6.0)
                ax.plot(self.current_deforms_x[m].traj, self.current_deforms_y[m].traj, self.current_deforms_z[m].traj, colors[m])
    cur_sim_vals = self.evaluate_repros()
    for m in range (self.n_algs):
        plt.figtext(0.7, 0.85 - (0.04 * m), self.alg_names[m] + ' raw similarity: ' + str(cur_sim_vals[m]), color=colors[m], wrap=True, fontsize='large')
        #plt.figtext(1, 1, self.alg_names[m] + ' raw similarity: ' + str(cur_sim_vals[m]), color=colors[m], ha='right', va='top', wrap=True)
    plt.show()
    plt.close('all')
  
  def evaluate_repros(self):
    cur_sim_vals = []
    for m in range (self.n_algs):
        if self.n_dims == 1:
            sim_val = 0
            for n in range (self.n_metrics):
                sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)))), np.hstack((np.reshape(self.current_deforms_x[m].traj, (self.traj_len, 1))))))
            print(('Raw similarity value of %f for ' + self.alg_names[m]) % (sim_val))
            cur_sim_vals.append(sim_val)
        if self.n_dims == 2:
            sim_val = 0
            for n in range (self.n_metrics):
                sim_val = sim_val + (self.metric_weights_norm[n] * self.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)), np.reshape(self.org_y, (self.traj_len, 1)))), np.hstack((np.reshape(self.current_deforms_x[m].traj, (self.traj_len, 1)), np.reshape(self.current_deforms_y[m].traj, (self.traj_len, 1))))))
            print(('Raw similarity value of %f for ' + self.alg_names[m]) % (sim_val))
            cur_sim_vals.append(sim_val)
        if self.n_dims == 3:
            sim_val = 0
            for n in range (self.n_metrics):
                sim_val = sim_val + (self.metric_weights_norm[n] * elf.metrics[n](np.hstack((np.reshape(self.org_x, (self.traj_len, 1)), np.reshape(self.org_y, (self.traj_len, 1)), np.reshape(self.org_z, (self.traj_len, 1)))), np.hstack((np.reshape(self.current_deforms_x[m].traj, (self.traj_len, 1)), np.reshape(self.current_deforms_y[m].traj, (self.traj_len, 1)), np.reshape(self.current_deforms_z[m].traj, (self.traj_len, 1))))))
            print(('Raw similarity value of %f for ' + self.alg_names[m]) % (sim_val))
            cur_sim_vals.append(sim_val)
    return cur_sim_vals
  
  def reproduce_optimal_at_point(self, coords, plot=False, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    opt_alg_num = int(self.clf.predict(np.array(coords)))
    opt_alg = self.algs[opt_alg_num]
    name='Optimal_reproduction_at' + str(coords)
    if self.n_dims == 1:
        opt_x = opt_alg(self.org_x, coords[0][0])
        if plot:
            plt.plot(self.org_x, 'k')
            plt.plot(opt_x, colors[opt_alg_num])
            if (mode == 'save'):
                plt.savefig(filepath + name + '.png')
            else:
                plt.show()
            plt.close('all')
        return opt_x
    if self.n_dims == 2:
        opt_x = opt_alg(self.org_x, coords[0][0])
        opt_y = opt_alg(self.org_y, coords[0][1])
        if plot: 
            plt.plot(self.org_x, self.org_y, 'k', linewidth=7)
            plt.plot(self.org_x[0], self.org_y[0], 'k*', markersize=20)
            plt.plot(self.org_x[-1], self.org_y[-1], 'k.', markersize=20)
            plt.plot(coords[0][0], coords[0][1], 'k+', markersize=20, mew=5)
            plt.plot(opt_x, opt_y, colors[opt_alg_num], linewidth=5) 
            plt.axis('off')
            if (mode == 'save'):
                plt.savefig(filepath + name + '.png')
            else:
                plt.show()
            plt.close('all')
        return [opt_x, opt_y]
    if self.n_dims == 3:
        opt_x = opt_alg(self.org_x, coords[0][0])
        opt_y = opt_alg(self.org_y, coords[0][1])
        opt_z = opt_alg(self.org_z, coords[0][2])
        if plot:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(self.org_x, self.org_y, self.org_z, 'k')
            ax.plot(opt_x, opt_y, opt_z, colors[opt_alg_num])
            if (mode == 'save'):
                plt.savefig(filepath + name + '.png')
            else:
                plt.show()
            plt.close('all')
        return [opt_x, opt_y, opt_z, self.alg_names[opt_alg_num]]
  
  def generate_svm_region(self, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    n_surf = 20
    name = 'SVM Similarity Region'
    if self.n_dims == 1:
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        fig = plt.figure()
        for t in range (len(xnew)):
            plt.plot(xnew[t], colors[int(self.clf.predict(np.array([[xnew[t]]])))] + '.')
        plt.plot(self.org_x[0], 'k*')
        plt.xticks(self.x_vals)                     
    if self.n_dims == 2:
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
        fig = plt.figure()
        for t in range (len(xnew)):
            for u in range (len(ynew)):
                coords = np.array([[xnew[t][0], ynew[u][0]]])
                #coords = np.array([[xnew[t], ynew[u]]])
                #print(coords)
                #print(self.clf.predict(coords))
                plt.plot(xnew[t], ynew[u], colors[int(self.clf.predict(coords))] + '.')
        plt.plot(self.org_x[0], self.org_y[0], 'k*')
        plt.xticks(self.x_vals)
        plt.yticks(self.y_vals)           
    if self.n_dims == 3:
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
        znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
        ax = plt.axes(projection='3d')
        for t in range (len(xnew)):
            for u in range (len(ynew)):
                for v in range (len(znew)):
                    if (int(self.clf.predict(np.array([[xnew[t], ynew[u], znew[v]]]))) == 1):
                        print([[xnew[t], ynew[u], znew[v]]])
                    ax.scatter(xnew[t], ynew[u], znew[v], c=colors[int(self.clf.predict(np.array([[xnew[t], ynew[u], znew[v]]])))])
        ax.scatter(self.org_x[0], self.org_y[0], self.org_z[0], c='k')
        plt.axis('off')
    if (mode == 'save'):
        plt.savefig(filepath + name + '.png')
    else:
        plt.show()
    plt.close('all')
  
  def svm_region_contour(self, mode='save', filepath='', plot_point=None, z_val=None):
    #colors = ['r', 'g', 'b', 'c', 'm', 'y']
    #colors = ['r', 'b', 'g', 'g', 'm', 'y', 'g', 'b']
    colors = ['r', 'b', 'b', 'g', 'm', 'y']
    n_surf = 1000
    name = 'SVM Similarity Contour'
    if self.n_dims == 1:
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        fig = plt.figure()
        for t in range (len(xnew)):
            plt.plot(xnew[t], colors[int(self.clf.predict(np.array([[xnew[t]]])))] + '.')
        plt.plot(self.org_x[0], 'k*')
        plt.xticks(self.x_vals)                     
    if self.n_dims == 2:
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
        fig = plt.figure()
        xx, yy = np.meshgrid(xnew, ynew)
        Z = self.clf.predict(np.c_[xx.ravel(), yy.ravel()])
        Z = Z.reshape(xx.shape)
        plt.contourf(xx, yy, Z, colors=colors, alpha=0.8)
        #for t in range (len(xnew)):
            #for u in range (len(ynew)):
                #coords = np.array([[xnew[t][0], ynew[u][0]]])
                #print(coords)
                #print(self.clf.predict(coords))
        #        plt.plot(xnew[t], ynew[u], colors[int(self.clf.predict(coords))] + '.')
        plt.plot(self.org_x[0], self.org_y[0], 'k*', markersize=30)
        if plot_point != None:
            plt.plot(plot_point[0], plot_point[1], 'k+', markersize=30, mew=5) 
            name = name + ' with reproduction at ' + str(plot_point)
        plt.xticks(self.x_vals)
        plt.yticks(self.y_vals)  
        plt.axis('off')
    if self.n_dims == 3:
        if (z_val == None):
            print('Please give a z index for the image slice')
            return
        name = name + ' at z=' + str(self.z_vals[z_val])
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
        znew = np.linspace(self.z_vals[z_val], self.z_vals[z_val], n_surf)
        fig = plt.figure()
        xx, yy = np.meshgrid(xnew, ynew)
        zz = np.ones(np.shape(xx)) * self.z_vals[z_val]
        Z = self.clf.predict(np.c_[xx.ravel(), yy.ravel(), zz.ravel()])
        Z = Z.reshape(xx.shape)
        plt.contourf(xx, yy, Z, colors=colors, alpha=0.8)
        #for t in range (len(xnew)):
            #for u in range (len(ynew)):
                #coords = np.array([[xnew[t][0], ynew[u][0]]])
                #print(coords)
                #print(self.clf.predict(coords))
        #        plt.plot(xnew[t], ynew[u], colors[int(self.clf.predict(coords))] + '.')
        if (z_val == math.floor(self.grid_size / 2)):
            plt.plot(self.org_x[0], self.org_y[0], 'k*', markersize=30)
        if plot_point != None:
            plt.plot(plot_point[0], plot_point[1], 'k+', markersize=30, mew=5) 
            name = name + ' with reproduction at ' + str(plot_point)
        plt.xticks(self.x_vals)
        plt.yticks(self.y_vals)  
        plt.axis('off')
    if (mode == 'save'):
        plt.savefig(filepath + name + '.png')
    else:
        plt.show()
    plt.close('all')
  
  def _convert_num_to_rgb(self, A):
    #colors = ['r', 'g', 'b', 'c', 'm', 'y']
    if self.n_dims == 1:
        B = np.zeros((self.grid_size, 3))
        for i in range (self.grid_size):
            if A[i] == 0:
                #red
                B[i][0] = 207
                B[i][1] = 113
                B[i][2] = 117
            elif A[i] == 1:
                #green
                B[i][0] = 119
                B[i][1] = 185
                B[i][2] = 134
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
        B = np.full((self.grid_size, self.grid_size, 3), 0)
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                if A[i][j] == 0:
                    #red
                    B[i][j][0] = 207
                    B[i][j][1] = 113
                    B[i][j][2] = 117
                elif A[i][j] == 1:
                    #green
                    B[i][j][0] = 119
                    B[i][j][1] = 185
                    B[i][j][2] = 134
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
                elif A[i][j] < 0:
                    B[i][j][0] = 0
                    B[i][j][1] = 0
                    B[i][j][2] = 0
                else:
                    print('Too many algorithms to represent color')
    if self.n_dims == 3:
        print('Too difficult to represent 3D gradient')
        return
    return B
  
  def _interpolate_grid(self):
    self.interps = []
    for n in range (self.n_metrics):
        alg_interps = []
        for m in range (self.n_algs):
            if (self.n_dims == 1):
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size))), self._get_array_of_sim_metrics(m)))
            if (self.n_dims == 2):
                #print(self.x_vals)
                #print(self.y_vals)
                #print(self._get_array_of_sim_metrics(n, m))
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size)), np.reshape(self.y_vals, (self.grid_size))), self._get_array_of_sim_metrics(m)))
            if (self.n_dims == 3):
                alg_interps.append(RegularGridInterpolator((np.reshape(self.x_vals, (self.grid_size)), np.reshape(self.y_vals, (self.grid_size)), np.reshape(self.z_vals, (self.grid_size))), self._get_array_of_sim_metrics(m)))
        self.interps.append(alg_interps)
  
  
  def show_3d_similarity(self, mode='save', filepath=''):
    n_surf = 20
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    for m in range(self.n_algs):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        #A = self._get_array_of_sim_metrics(m)
        xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
        ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
        znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
        #for i in range (self.grid_size):
        #    for j in range (self.grid_size):
        #        for k in range (self.grid_size):
        #            ax.plot([self.x_vals[i]], [self.y_vals[j]], [self.z_vals[k]], colors[m] + '.', alpha=A[i][j][k])
        plt.axis('off')
        for i in range(n_surf):
            for j in range(n_surf):
                for k in range(n_surf):
                    res = self.interps[0][m](np.array([xnew[i], ynew[j], znew[k]]).reshape(self.n_dims))
                    print(res)
                    ax.plot([xnew[i]], [ynew[j]], [znew[k]], colors[m] + '.', alpha=res[0])
        if (mode == 'save'):
            plt.savefig(filepath + '3d_similarity_Plot.png')
        else:
            plt.show()
  
  def show_3d_similarity_optimal(self, mode='save', filepath=''):
    self._interpolate_grid()
    n_surf = 20
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #A = self._get_array_of_sim_metrics(m)
    xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
    ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
    znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
    #for i in range (self.grid_size):
    #    for j in range (self.grid_size):
    #        for k in range (self.grid_size):
    #            ax.plot([self.x_vals[i]], [self.y_vals[j]], [self.z_vals[k]], colors[m] + '.', alpha=A[i][j][k])
    plt.axis('off')
    res = np.zeros((3));
    for i in range(n_surf):
        for j in range(n_surf):
            for k in range(n_surf):
                for m in range(self.n_algs):
                    res[m] = self.interps[0][m](np.array([xnew[i], ynew[j], znew[k]]).reshape(self.n_dims))
                val = max(res)
                ind = np.argmax(res)
                print(val)
                ax.plot([xnew[i]], [ynew[j]], [znew[k]], colors[ind] + '.', alpha=val)
    if (mode == 'save'):
        plt.savefig(filepath + '3d_similarity_Plot_Optimal.png')
    else:
        plt.show()
  
  def get_image_slices(self, mode='save', filepath=''):
    for i in range(self.grid_size):
        self.svm_region_contour(mode=mode, filepath=filepath, z_val=i)
    #colors = ['r', 'g', 'b', 'c', 'm', 'y']
    #n_surf = 200;
    #xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
    #ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
    #for k in range(self.grid_size):
    #    fig = plt.figure;
    #    name = 'SVM Similarity at z=' + str(self.z_vals[k])
    #    for i in range(n_surf):
    #        for j in range(n_surf):
    #            coords = [[xnew[i], ynew[j], self.z_vals[k]]]
    #            opt_alg_num = int(self.clf.predict(np.array(coords)))
    #            #plt.plot(xnew[i], ynew[j], colors[opt_alg_num] + '.', markersize=1)
    #            #plt.xticks(self.x_vals)
    #            #plt.yticks(self.y_vals)  
    #            plt.axis('off')
    #    plt.contourf(xx, yy, Z, colors=colors, alpha=0.8)
    #    if (mode == 'save'):
    #        plt.savefig(filepath + name + '.png')
    #    else:
    #        plt.show()
    #plt.close('all')
    
  def plot_surfaces(self, mode='save', filepath=''):
    self._interpolate_grid()
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
        self._plot_xy_surf(mode, filepath)
        self._plot_xz_surf(mode, filepath)
        self._plot_yz_surf(mode, filepath)
    plt.close('all')
    
  def _plot_xy_surf(self, mode='save', filepath=''):
    self._interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        for m in range (self.n_algs):  
            if self.n_dims == 3:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                X, Y = np.meshgrid(xnew, ynew)
                plt_new = np.zeros((len(xnew), len(ynew)))
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        arr = np.array([xnew[t], ynew[u], self.org_z[0]]).reshape(self.n_dims)
                        plt_new[t][u] = self.interps[n][m](arr)
                fig = plt.figure()
                ax = plt.axes(projection='3d')
                ax.plot_surface(X, Y, plt_new, cmap='viridis', edgecolor='none')
                name = self.alg_names[m] + ' ' + self.metric_names[n] + ' XY Surface'
                ax.set_title(name, fontsize=self.f_size)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()
    plt.close('all')
    
  def _plot_xz_surf(self, mode='save', filepath=''):
    self._interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        for m in range (self.n_algs):  
            if self.n_dims == 3:
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
                X, Z = np.meshgrid(xnew, znew)
                plt_new = np.zeros((len(xnew), len(znew)))
                for t in range (len(xnew)):
                    for u in range (len(znew)):
                        arr = np.array([xnew[t], self.org_y[0], znew[u]]).reshape(self.n_dims)
                        plt_new[t][u] = self.interps[n][m](arr)
                fig = plt.figure()
                ax = plt.axes(projection='3d')
                ax.plot_surface(X, Z, plt_new, cmap='viridis', edgecolor='none')
                name = self.alg_names[m] + ' ' + self.metric_names[n] + ' XZ Surface'
                ax.set_title(name, fontsize=self.f_size)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()
    plt.close('all')
    
  def _plot_yz_surf(self, mode='save', filepath=''):
    self._interpolate_grid()
    n_surf = 100
    for n in range (self.n_metrics):
        for m in range (self.n_algs):  
            if self.n_dims == 3:
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
                Y, Z = np.meshgrid(ynew, znew)
                plt_new = np.zeros((len(ynew), len(znew)))
                for t in range (len(ynew)):
                    for u in range (len(znew)):
                        arr = np.array([self.org_x[0], ynew[t], znew[u]]).reshape(self.n_dims)
                        plt_new[t][u] = self.interps[n][m](arr)
                fig = plt.figure()
                ax = plt.axes(projection='3d')
                ax.plot_surface(Y, Z, plt_new, cmap='viridis', edgecolor='none')
                name = self.alg_names[m] + ' ' + self.metric_names[n] + ' YZ Surface'
                ax.set_title(name, fontsize=self.f_size)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()
    plt.close('all')
    
  def _get_array_of_sim_metrics_old(self, metric_num, alg_num):
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

  def _get_array_of_sim_metrics(self, alg_num):
    if self.n_dims == 1:
        A = np.zeros((self.grid_size))
        for i in range (self.grid_size):
            A[i] = self.grid_similarities[i][alg_num].val                                
    if self.n_dims == 2:
        A = np.zeros((self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                A[i][j] = self.grid_similarities[i][j][alg_num].val
    if self.n_dims == 3:
        A = np.zeros((self.grid_size, self.grid_size, self.grid_size))
        for i in range (self.grid_size):
            for j in range (self.grid_size):
                for k in range (self.grid_size):
                    A[i][j][k] = self.grid_similarities[i][j][k][alg_num].val
    return A

  def get_plots_at_point1(self, i, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    plt.plot(self.org_x, 'k')
    for m in range (self.n_algs):
        plt.plot(self.grid_deforms_x[i][m].traj, colors[m])
    plt.title('Deformations at grid point (' + str(i) + ')', fontsize=self.f_size)
    if (mode == 'save'):
        plt.savefig(filepath + 'Deformations at grid point (' + str(i) + ').png')
    else:
        plt.show()   
  
  def get_plots_at_point2(self, i, j, alg=-1, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    fig = plt.figure()
    plt.plot(self.org_x, self.org_y, 'k', linewidth=6.0)
    plt.plot(self.org_x[0], self.org_y[0], 'k*', markersize=20)
    plt.plot(self.org_x[self.traj_len - 1], self.org_y[self.traj_len - 1], 'ko', markersize=20)
    for m in range (self.n_algs):
        if (m == alg):
            plt.plot(self.grid_deforms_x[i][j][m].traj, self.grid_deforms_y[i][j][m].traj, colors[m], linewidth=6.0)
        else:
            plt.plot(self.grid_deforms_x[i][j][m].traj, self.grid_deforms_y[i][j][m].traj, colors[m])  
        plt.plot(self.grid_deforms_x[i][j][m].traj[0], self.grid_deforms_y[i][j][m].traj[0], colors[m] + '+', markersize=20)
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
    for m in range (self.n_algs):
        ax.plot(self.grid_deforms_x[i][j][k][m].traj, self.grid_deforms_y[i][j][k][m].traj, self.grid_deforms_z[i][j][k][m].traj, colors[m])
    plt.title('Deformations at grid point (' + str(i) + ', ' + str(j) + ', ' + str(k) + ')', fontsize=self.f_size)
    if (mode == 'save'):
        plt.savefig(filepath + 'Deformations at grid point (' + str(i) + ', ' + str(j) + ', ' + str(k) + ').png')
    else:
        plt.show()   
          
  def plot_sim(self, sim, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    self._interpolate_grid()
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
                xnew = np.linspace(self.x_vals[0], self.x_vals[self.grid_size - 1], n_surf)
                ynew = np.linspace(self.y_vals[0], self.y_vals[self.grid_size - 1], n_surf)
                znew = np.linspace(self.z_vals[0], self.z_vals[self.grid_size - 1], n_surf)
                ax = plt.axes(projection='3d')
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        for v in range (len(znew)):
                            #print('m: %d t: %d u: %d' % (m, t, u))
                            arr = np.array([xnew[t], ynew[u], znew[v]]).reshape(self.n_dims)
                            if self.interps[n][m](arr) > sim:
                                ax.scatter(xnew[t], ynew[u], c=colors[m])
                name = self.metric_names[n] + ' similarity of ' + str(sim) + ' for ' + self.alg_names[m] + ' Plot'
                print(name)
                #plt.title(name, fontsize=self.f_size)
                ax.scatter(self.org_x[0], self.org_y[0], self.org_z[0], c='k')
                ax.set_xticks(self.x_vals)
                ax.set_yticks(self.y_vals)
                ax.set_zticks(self.z_vals)
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()
    plt.close('all')
    
  def plot_sim_hmap(self, mode='save', filepath=''):
    #colors = ['r', 'g', 'b', 'c', 'm', 'y']
    self._interpolate_grid()
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
                A = np.zeros((n_surf, n_surf))
                for t in range (len(xnew)):
                    for u in range (len(ynew)):
                        #print('m: %d t: %d u: %d' % (m, t, u))
                        arr = np.array([xnew[t], ynew[u]]).reshape(self.n_dims)
                        #if self.interps[n][m](arr) > sim:
                        #    plt.plot(xnew[t], ynew[u], colors[m] + '.')
                        A[t][u] = self.interps[n][m](arr)
                ax = sns.heatmap(np.transpose(A), annot=False)
                name = self.metric_names[n] + ' for ' + self.alg_names[m] + ' Heatmap'
                print(name)
                #plt.title(name, fontsize=self.f_size)
                #plt.plot(self.org_x[0], self.org_y[0], 'k*')
                plt.xticks([])
                plt.yticks([])
                if (mode == 'save'):
                    plt.savefig(filepath + name + '.png')
                else:
                    plt.show()           
            if self.n_dims == 3:
                print('Similarity plot in 3D too difficult to show')
    plt.close('all')
    
  def plot_sim_strongest(self, sim, mode='save', filepath=''):
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    self._interpolate_grid()
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
            #mlfd(x_data, y_data, shape_names[i] + '_dmp_on', is_dmp_on=True)
            mlfd(x_data, y_data, shape_names[i], is_dmp_on=False)

def my_hd2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((len(x1), 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    return max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
    
def my_hd3(x1, x2, y1, y2, z1, z2):
    org_traj = np.zeros((len(x1), 3))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1)
    org_traj[:, 1] = np.transpose(y1)
    org_traj[:, 2] = np.transpose(z1)
    comp_traj[:, 0] = np.transpose(x2)
    comp_traj[:, 1] = np.transpose(y2)
    comp_traj[:, 2] = np.transpose(z2)
    return max(directed_hausdorff(org_traj, comp_traj)[0], directed_hausdorff(comp_traj, org_traj)[0])
    
def my_fd2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((len(x1), 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    return similaritymeasures.frechet_dist(org_traj, comp_traj)

def my_fd3(x1, x2, y1, y2, z1, z2):
    org_traj = np.zeros((len(x1), 3))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1)
    org_traj[:, 1] = np.transpose(y1)
    org_traj[:, 2] = np.transpose(z1)
    comp_traj[:, 0] = np.transpose(x2)
    comp_traj[:, 1] = np.transpose(y2)
    comp_traj[:, 2] = np.transpose(z2)
    return similaritymeasures.frechet_dist(org_traj, comp_traj)

def my_dtw2(x1, x2, y1, y2):
    x_DTW = dtw.dtw(x1, x2)
    y_DTW = dtw.dtw(y1, y2)
    return (x_DTW.normalizedDistance + y_DTW.normalizedDistance) / 2.0
    
def my_curvature_conservation2(x1, x2, y1, y2):
    sse = 0
    numel = len(x1)
    x1 = np.transpose(x1).reshape((numel))
    y1 = np.transpose(y1).reshape((numel))
    x2 = np.transpose(x2).reshape((numel))
    y2 = np.transpose(y2).reshape((numel))
    for i in range (1,numel - 1):
        d2y1 = y1[i - 1] - (2 * y1[i]) + y1[i + 1]
        dx21 = (x1[i + 1] - x1[i - 1])**2
        d2ydx21 = d2y1 / dx21
        d2y2 = y2[i - 1] - (2 * y2[i]) + y2[i + 1]
        dx22 = (x2[i + 1] - x2[i - 1])**2
        d2ydx22 = d2y2 / dx22
        error = d2ydx21 - d2ydx22
        sse = sse + error**2
    return sse
 
def my_crv2(x1, x2, y1, y2):
    sse = 0
    numel = len(x1)
    x1 = np.transpose(x1).reshape((numel))
    y1 = np.transpose(y1).reshape((numel))
    x2 = np.transpose(x2).reshape((numel))
    y2 = np.transpose(y2).reshape((numel))
    L = 2.*np.diag(np.ones((numel,))) - np.diag(np.ones((numel-1,)),1) - np.diag(np.ones((numel-1,)),-1)
    L[0,1] = -2.
    L[-1,-2] = -2.
    x_err = np.subtract(np.matmul(L, x1), np.matmul(L, x2))
    y_err = np.subtract(np.matmul(L, y1), np.matmul(L, y2))
    return np.sum(np.power(x_err, 2)) + np.sum(np.power(y_err, 2))

def my_jerk2(x1, x2, y1, y2):
    numel = len(x1)
    x1 = np.transpose(x1).reshape((numel))
    y1 = np.transpose(y1).reshape((numel))
    x2 = np.transpose(x2).reshape((numel))
    y2 = np.transpose(y2).reshape((numel))
    J = -3.*np.diag(np.ones((numel,))) + 3.*np.diag(np.ones((numel-1,)), 1) - np.diag(np.ones((numel-2,)),2) + np.diag(np.ones((numel-1,)),-1)
    x_err = np.subtract(np.matmul(J, x1), np.matmul(J, x2))
    y_err = np.subtract(np.matmul(J, y1), np.matmul(J, y2))
    return np.sum(np.power(x_err, 2)) + np.sum(np.power(y_err, 2))

def my_endpoint_convergence2(x1, x2, y1, y2):
    numel = len(x1)
    x1 = np.transpose(x1).reshape((numel))
    y1 = np.transpose(y1).reshape((numel))
    x2 = np.transpose(x2).reshape((numel))
    y2 = np.transpose(y2).reshape((numel))
    return get_euclidian_dist(x2[-1], x1[-1], y2[-1], y1[-1])

def my_curve_length2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((numel, 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    print('Theirs:')
    print(similaritymeasures.curve_length_measure(org_traj, comp_traj))
    return similaritymeasures.curve_length_measure(org_traj, comp_traj)

def my_pcm2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((numel, 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    return similaritymeasures.pcm(org_traj, comp_traj)

def my_area_eval2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((numel, 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    return similaritymeasures.area_between_two_curves(org_traj, comp_traj)

def sim_measure_dtw2(x1, x2, y1, y2):
    numel = len(x1)
    org_traj = np.zeros((numel, 2))
    comp_traj = np.zeros((np.shape(org_traj)))
    org_traj[:, 0] = np.transpose(x1).reshape((numel))
    org_traj[:, 1] = np.transpose(y1).reshape((numel))
    comp_traj[:, 0] = np.transpose(x2).reshape((numel))
    comp_traj[:, 1] = np.transpose(y2).reshape((numel))
    dtw, d = similaritymeasures.dtw(org_traj, comp_traj)
    return dtw

def get_total_dist(x, y):
    ttl = 0
    for i in range (len(x) - 2):
        ttl = ttl + get_euclidian_dist(x[i], y[i], x[i+1], y[i+1])
    return ttl
    
def total_distance_comp2(x1, x2, y1, y2):
    print('Mine:')
    print(abs(get_total_dist(x1, y1) - get_total_dist(x2, y2)))
    return abs(get_total_dist(x1, y1) - get_total_dist(x2, y2))

def get_euclidian_dist_any(pts1, pts2):
    n_dims = len(pts1)
    sum = 0.
    for i in range(n_dims):
        sum += (pts1[i] - pts2[i])**2
    return sum**0.5
    
def sum_of_dists(exp_data, num_data):
    #naive approach
    (n_points, n_dims) = np.shape(exp_data)
    if not np.shape(exp_data) == np.shape(num_data):
        print('Array dims must match!')
    sum = 0.
    for i in range(n_points):
        sum += get_euclidian_dist_any(exp_data[i], num_data[i]) 
    return sum

def curvature_comparison(exp_data, num_data):
    (n_points, n_dims) = np.shape(exp_data)
    if not np.shape(exp_data) == np.shape(num_data):
        print('Array dims must match!')
    L = 2.*np.diag(np.ones((n_points,))) - np.diag(np.ones((n_points-1,)),1) - np.diag(np.ones((n_points-1,)),-1)
    L[0,1] = -2.
    L[-1,-2] = -2.
    err_abs = np.absolute(np.subtract(np.matmul(L, exp_data), np.matmul(L, num_data)))
    return np.sum(err_abs)

def herons_formula(a, b, c):
    s = (a + b + c) / 2.
    return (s * (s - a) * (s - b) * (s - c))**0.5

def swept_error_area(exp_data, num_data):
    #naive approach
    (n_points, n_dims) = np.shape(exp_data)
    if not np.shape(exp_data) == np.shape(num_data):
        print('Array dims must match!')
    sum = 0.
    for i in range(n_points - 1):
        p1 = exp_data[i]
        p2 = exp_data[i + 1]
        p3 = num_data[i + 1]
        p4 = num_data[i]
        p1_p2_dist = get_euclidian_dist_any(p1, p2)
        p1_p3_dist = get_euclidian_dist_any(p1, p3)
        p1_p4_dist = get_euclidian_dist_any(p1, p4)
        p2_p3_dist = get_euclidian_dist_any(p2, p3)
        p3_p4_dist = get_euclidian_dist_any(p3, p4)
        triangle1_area = herons_formula(p1_p4_dist, p1_p3_dist, p3_p4_dist)
        triangle2_area = herons_formula(p1_p2_dist, p1_p3_dist, p2_p3_dist)
        sum += triangle1_area + triangle2_area
    return sum / n_points

def sum_of_squared_error(exp_data, num_data):
    #naive approach
    (n_points, n_dims) = np.shape(exp_data)
    if not np.shape(exp_data) == np.shape(num_data):
        print('Array dims must match!')
    sum = 0.
    for i in range(n_points):
        sum += (get_euclidian_dist_any(exp_data[i], num_data[i]))**2
    return sum

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
    my_mlfd = mlfd()
    my_mlfd.add_traj_dimension(x_data, 'x')
    my_mlfd.add_traj_dimension(y_data, 'y')
    #print(x_data)
    #print(y_data)
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_sim_metric(my_fd2, name='Frechet', is_disssim=True)
    my_mlfd.add_sim_metric(my_hd2, name='Haussdorf', is_disssim=True)
    #my_mlfd.add_traj_dimension(x_data, 'z')
    my_mlfd.create_grid(3, [20, 20])
    my_mlfd.deform_traj(plot=False)
    my_mlfd.calc_metrics(d_sample=True)
    #my_mlfd.plot_gradients(mode='show', filepath=plt_fpath)
    #my_mlfd.plot_gradients(mode='save', filepath=plt_fpath)
    #my_mlfd.plot_strongest_gradients(mode='show', filepath=plt_fpath)
    #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
    my_mlfd.plot_surfaces(mode='show', filepath=plt_fpath)
    my_mlfd.plot_surfaces(mode='save', filepath=plt_fpath)
    my_mlfd.get_plots_at_point2(0, 1)
    my_mlfd.get_plots_at_point2(2, 1)
    
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
            plt_fpath = '../pictures/lte_writing/mlfd/' + shape_names[i] + '/'
            try:
                os.makedirs(plt_fpath)
            except OSError:
                print ("Creation of the directory %s failed" % plt_fpath)
            else:
                print ("Successfully created the directory %s" % plt_fpath)
            my_mlfd = mlfd()
            my_mlfd.add_traj_dimension(x_data, 'x')
            my_mlfd.add_traj_dimension(y_data, 'y')
            my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
            my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
            my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
            my_mlfd.add_sim_metric(my_fd2, name='Frechet', is_disssim=True)
            my_mlfd.add_sim_metric(my_hd2, name='Haussdorf', is_disssim=True)
            #my_mlfd.add_traj_dimension(x_data, 'z')
            my_mlfd.create_grid(10, [20, 20])
            my_mlfd.deform_traj(plot=False)
            my_mlfd.calc_metrics(d_sample=True)
            my_mlfd.save_results('straight_ribbon_deform_data.h5')
            #my_mlfd.plot_gradients(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_gradients(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_strongest_gradients(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_surfaces(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_surfaces(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_sim(sim=0.9, mode='show')
            #my_mlfd.get_plots_at_point2(0, 2, mode='show')
            #my_mlfd.get_plots_at_point2(5, 0, mode='show')
            #my_mlfd.get_plots_at_point2(6, 7, mode='show')
            
def main5():
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
    #fig = plt.figure()
    #plt.plot(x_data, y_data, 'k', linewidth=6.0)
    #plt.plot(x_data[0], y_data[0], 'k*', markersize=20)
    #plt.plot(x_data[len(x_data) - 1], y_data[len(y_data) - 1], 'ko', markersize=20)
    #plt.show()
    my_mlfd = mlfd()
    my_mlfd.add_traj_dimension(x_data, 'x')
    my_mlfd.add_traj_dimension(y_data, 'y')
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    my_mlfd.add_sim_metric(my_fd2, name='Frechet', is_dissim=True)
    my_mlfd.add_sim_metric(my_hd2, name='Haussdorf', is_dissim=True)
    #my_mlfd.add_traj_dimension(x_data, 'z')
    #my_mlfd.create_grid(10, [20, 20])
    #my_mlfd.create_grid(5, [20, 20])
    #my_mlfd.deform_traj(plot=False)
    #my_mlfd.calc_metrics(d_sample=True)
    #my_mlfd.save_results('straight_ribbon_deform_data_full.h5')
    my_mlfd.read_from_h5('straight_ribbon_deform_data_full.h5')
    #my_mlfd.plot_sim_hmap(mode='show')
    #my_mlfd.plot_heatmap(mode='show')
    #my_mlfd.plot_heatmap(mode='save', filepath=plt_fpath)
    #my_mlfd.plot_gradients(mode='show')
    #my_mlfd.plot_gradients(mode='save', filepath=plt_fpath)
    #my_mlfd.plot_strongest_gradients(mode='show')
    #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
    #my_mlfd.plot_surfaces(mode='show', filepath=plt_fpath)
    #my_mlfd.plot_surfaces(mode='save', filepath=plt_fpath)
    #print(my_mlfd.get_array_of_sim_metrics(0, 0))
    #print('input:')
    #a = float(input())
    #my_mlfd.plot_sim_strongest(sim=0.72, mode='show')
    my_mlfd.get_plots_at_point2(7, 7, alg=0, mode='show')
    #my_mlfd.get_plots_at_point2(5, 0, alg=2, mode='show')
    #my_mlfd.get_plots_at_point2(6, 7, alg=1, mode='show')
    
def main6():
    #shape_names = ['Circle', 'Infinity', 'Pi', 'Pyramids', 'Ribbon', 'Slanted_Square', 'Spiral', 'Straight_Ribbon', 'Three', 'Worm']
    #shape_names = ['Pi', 'Spiral']
    shape_names = ['Spiral']
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
            plt_fpath = '../pictures/lte_writing/mlfd/' + shape_names[i] + '/'
            try:
                os.makedirs(plt_fpath)
            except OSError:
                print ("Creation of the directory %s failed" % plt_fpath)
            else:
                print ("Successfully created the directory %s" % plt_fpath)
            my_mlfd = mlfd()
            my_mlfd.add_traj_dimension(x_data, 'x')
            my_mlfd.add_traj_dimension(y_data, 'y')
            my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
            my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
            my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
            my_mlfd.add_sim_metric(my_fd2, name='Frechet', is_dissim=True)
            my_mlfd.add_sim_metric(my_hd2, name='Haussdorf', is_dissim=True)
            #my_mlfd.add_traj_dimension(x_data, 'z')
            my_mlfd.create_grid(10, [20, 20])
            my_mlfd.deform_traj(plot=True)
            my_mlfd.calc_metrics(d_sample=True)
            my_mlfd.save_results(plt_fpath + 'mlfd_data.h5')
            #my_mlfd.plot_gradients(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_gradients(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_strongest_gradients(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_surfaces(mode='show', filepath=plt_fpath)
            #my_mlfd.plot_surfaces(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_sim(sim=0.9, mode='show')
            #my_mlfd.get_plots_at_point2(0, 2, mode='show')
            #my_mlfd.get_plots_at_point2(5, 0, mode='show')
            #my_mlfd.get_plots_at_point2(6, 7, mode='show')
    
def main():
    shape_names = 'Spiral'
    filename = '../h5 files/' + shape_names +'_drawing_demo.h5'
    hf = h5py.File(filename, 'r')
    demo = hf.get(shape_names)
    x_data = demo.get('x')
    x_data = np.array(x_data)
    y_data = demo.get('y')
    y_data = np.array(y_data)
    hf.close()
    plt_fpath = '../pictures/lte_writing/mlfd/' + shape_names + '/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
    my_mlfd = mlfd()
    my_mlfd.add_traj_dimension(x_data, 'x')
    my_mlfd.add_traj_dimension(y_data, 'y')
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    my_mlfd.use_default_metrics()
    #my_mlfd.create_grid(9, [20, 20])
    #my_mlfd.deform_traj(plot=False)
    #my_mlfd.calc_metrics(d_sample=False)
    #my_mlfd.save_results('spiral_2d_svm_testing.h5')
    my_mlfd.read_from_h5('spiral_2d_svm_testing.h5')
    my_mlfd.set_up_classifier()
    #e = 10
    #my_mlfd.query_svm(np.array([[x_data[0][0] - e, y_data[0][0] + e]]), plot=True)
    my_mlfd.svm_region_contour(mode='show')
    
    
if __name__ == '__main__':
  main()