import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from sklearn import mixture
import h5py
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp2d

#open the file
filename = '../h5 files/xy_fd_hd_grid10.h5'
hf = h5py.File(filename, 'r')
#navigate to necessary data and store in numpy arrays
x = hf.get('x')
x = np.array(x)
y = hf.get('y')
y = np.array(y)
ja_fd = hf.get('ja_fd')
ja_fd = np.array(ja_fd)
ja_hd = hf.get('ja_hd')
ja_hd = np.array(ja_hd)
lte_fd = hf.get('lte_fd')
lte_fd = np.array(lte_fd)
lte_hd = hf.get('lte_hd')
lte_hd = np.array(lte_hd)
dmp_fd = hf.get('dmp_fd')
dmp_fd = np.array(dmp_fd)
dmp_hd = hf.get('dmp_hd')
dmp_hd = np.array(dmp_hd)
#close out file
hf.close()

grid_size = np.shape(x)[0];

x_vals = x[:, 0]
y_vals = y[0, :]

z = interp2d(x_vals, y_vals, 1.0 / lte_fd)

xnew = np.linspace(x_vals[0], x_vals[grid_size - 1], 1000)
ynew = np.linspace(y_vals[0], y_vals[grid_size - 1], 1000)
znew = z(xnew, ynew)
#print(x_vals)
#print(y_vals)

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
ax = plt.axes(projection='3d')
#print(ja_hd)
#ax.plot(x_vals, y_vals, ja_hd)
#for i in range (grid_size):
    #for j in range (grid_size):    
        #ax.scatter(x_vals[i], y_vals[j], ja_hd[i, j], c='b')
        #ax.scatter(x_vals[i], y_vals[j], ja_fd[i, j], c='r')
        #ax.scatter(x_vals[i], y_vals[j], dmp_hd[i, j], c='g')
        #ax.scatter(x_vals[i], y_vals[j], dmp_fd[i, j], c='c')
        #ax.scatter(x_vals[i], y_vals[j], lte_hd[i, j], c='m')
        #ax.contour3D(x_vals, y_vals, lte_fd, 50, cmap='binary')
ax.contour3D(xnew, ynew, znew, 50, cmap='binary')       
        
plt.show()