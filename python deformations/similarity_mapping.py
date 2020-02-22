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


#in-file testing
def main():
    is_dmp_on = True
    ## Get Trajectory ##
    print('Getting Trajectory')
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    x_data = np.array(x_data)
    y_data = hello.get('resampled_y')
    y_data = np.array(y_data)
    hf.close()
    ## Optimize JA for trajectory ##
    print('Optimizing JA')
    lambda_x = optimize_ja.opt_lambda_traj_1d(x_data)
    lambda_y = optimize_ja.opt_lambda_traj_1d(y_data)
    ## Get deform grid ##
    print('Getting Deform Grid')
    #Constants--can be changed
    grid_x_dist = 0.5
    grid_y_dist = 0.5
    grid_size = 5;
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
    print(filename + '_grid' + str(grid_size) + '.h5')
    fp = h5py.File (filename + '_grid' + str(grid_size) + '.h5', 'w')
    dset_name = 'hello'
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
    plt.show()
    #store hd/fd data in h5
    fp.create_dataset(dset_name + '/lte/fd', data=fd_lte)
    fp.create_dataset(dset_name + '/lte/hd', data=hd_lte)
    fp.create_dataset(dset_name + '/ja/fd', data=fd_ja)
    fp.create_dataset(dset_name + '/ja/hd', data=hd_ja)
    if is_dmp_on:
        fp.create_dataset(dset_name + '/dmp/fd', data=fd_dmp)
        fp.create_dataset(dset_name + '/dmp/hd', data=hd_dmp)
    #gradient maps
    gradient_plotting.gradient_map(fd_lte, 'LTE Frechet Distance')
    gradient_plotting.gradient_map(hd_lte, 'LTE Haussdorf Distance')
    gradient_plotting.gradient_map(fd_ja, 'JA Frechet Distance')
    gradient_plotting.gradient_map(hd_ja, 'JA Haussdorf Distance')
    if is_dmp_on:
        gradient_plotting.gradient_map(fd_dmp, 'DMP Frechet Distance')
        gradient_plotting.gradient_map(hd_dmp, 'DMP Haussdorf Distance')
        gradient_plotting.rgb_gradient(fd_ja, fd_lte, fd_dmp, name='Frechet Distance Compared Reproductions')
        gradient_plotting.rgb_gradient(hd_ja, hd_lte, hd_dmp, name='Haussdorf Distance Compared Reproductions')
        gradient_plotting.strongest_gradient(fd_ja, fd_lte, fd_dmp, name='Frechet Distance Best Reproductions')
        gradient_plotting.strongest_gradient(hd_ja, hd_lte, hd_dmp, name='Haussdorf Distance Best Reproductions')
    gradient_plotting.rgb_gradient(fd_ja, fd_lte, np.zeros((np.shape(fd_lte))), name='Frechet Distance Compared Reproductions')
    gradient_plotting.rgb_gradient(hd_ja, hd_lte, np.zeros((np.shape(fd_lte))), name='Haussdorf Distance Compared Reproductions')
    gradient_plotting.strongest_gradient(fd_ja, fd_lte, np.zeros((np.shape(fd_lte))), name='Frechet Distance Best Reproductions')
    gradient_plotting.strongest_gradient(hd_ja, hd_lte, np.zeros((np.shape(fd_lte))), name='Haussdorf Distance Best Reproductions')
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
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    #plot all three on a single plot?
    #have all seperate plots?
    #how to show which sirface is better at a single point?
    #different color maps
    #how do I show hd vs. fd?
    f_size = 32
    ax.plot_surface(X, Y, fd_lte_plot,cmap='viridis', edgecolor='none')
    ax.set_title('LTE Frechet Distance', fontsize=f_size)
    plt.show()
    ax.plot_surface(X, Y, hd_lte_plot,cmap='viridis', edgecolor='none')
    ax.set_title('LTE Haussdorf Distance', fontsize=f_size)
    plt.show()
    ax.plot_surface(X, Y, fd_ja_plot,cmap='viridis', edgecolor='none')
    ax.set_title('JA Frechet Distance', fontsize=f_size)
    plt.show()
    ax.plot_surface(X, Y, hd_ja_plot,cmap='viridis', edgecolor='none')
    ax.set_title('JA Haussdorf Distance', fontsize=f_size)
    plt.show()
    if is_dmp_on:
        ax.plot_surface(X, Y, fd_dmp_plot,cmap='viridis', edgecolor='none')
        ax.set_title('DMP Frechet Distance', fontsize=f_size)
        plt.show()
        ax.plot_surface(X, Y, hd_dmp_plot,cmap='viridis', edgecolor='none')
        ax.set_title('DMP Haussdorf Distance', fontsize=f_size)
        plt.show()
    fp.close()

if __name__ == '__main__':
  main()