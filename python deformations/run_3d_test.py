import mlfd
import numpy as np
import h5py
import preprocess
import ja
import lte
import similaritymeasures
import os
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'C:/Users/BH/Documents/GitHub/Pearl-ur5e_matlab-implementations/python deformations/dmp_for_comparison/')

import perform_new_dmp as pnd

#good_demos = [['PUSHING', 'file1', 'demo2'], ['PUSHING', 'file4', 'demo2'], ['PUSHING', 'file7', 'demo2'], ['REACHING', 'file1', 'demo6'], ['REACHING', 'file3', 'demo2'], ['REACHING', 'file3', 'demo6']]

test_demo = ['Reaching', 'file3', 'demo6']

def main():
    ## get data fromm h5 file ##
    skill = test_demo[0]
    filename = '../h5 files/' + skill + '_dataset.h5'
    #open the file
    hf = h5py.File(filename, 'r')
    print(hf)
    #navigate to necessary data and store in numpy arrays
    fname = test_demo[1]
    print(fname)
    f = hf.get(fname)
    print(f)
    dname = test_demo[2]
    print(dname)
    d = f.get(dname)
    print(d)
    pos = d.get('pos')
    
    pos_arr = np.array(pos)
    n_dsample = 300
    x = mlfd.downsample_1d(pos[0], n_dsample)
    y = mlfd.downsample_1d(pos[1], n_dsample)
    z = mlfd.downsample_1d(pos[2], n_dsample)
    hf.close()
    
    plt_fpath = '3d_reproduction_testing/' + skill + '/' + fname + '/' + dname + '/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
        
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, 'k', linewidth=10)
    ax.plot([x[0]], [y[0]], [z[0]],  'k*', markersize=40)
    ax.plot([x[-1]], [y[-1]], [z[-1]],  'k.', markersize=40)
    plt.axis('off')
    plt.savefig(plt_fpath + 'demonstration.png')
    plt.close('all')
    
    ## set up mlfd ##
    my_mlfd = mlfd.mlfd()
    my_mlfd.add_traj_dimension(x, 'x')
    my_mlfd.add_traj_dimension(y, 'y')
    my_mlfd.add_traj_dimension(z, 'z')
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(pnd.perform_new_dmp_adapted, 'DMP')
    my_mlfd.add_metric(similaritymeasures.frechet_dist, name='Frechet', is_dissim=True)
    
    ## first run params ##
    #my_mlfd.create_grid()
    #my_mlfd.deform_traj()
    #plt.close('all')
    #my_mlfd.calc_metrics()
    
    ## save/load from file ##
    #my_mlfd.save_results(plt_fpath + skill + '_mlfd.h5')
    my_mlfd.read_from_h5(plt_fpath + skill + '_mlfd.h5')
    
    ## interpret results ##
    my_mlfd.set_up_classifier()
    #my_mlfd.generate_svm_region(mode='save', filepath=plt_fpath)
    #my_mlfd.get_image_slices(mode='save', filepath=plt_fpath)
    #my_mlfd.show_3d_similarity_optimal(mode='save', filepath=plt_fpath)
    dist = my_mlfd.get_demo_dist()
    [opt_x, opt_y, opt_z, alg_name] = my_mlfd.reproduce_optimal_at_point([[x[0] + dist / 12., y[0] - dist / 14., z[0] + dist / 16.]], plot=True, mode='save', filepath=plt_fpath) 
    fp = h5py.File(skill + '_' + alg_name + '_3D_reproduction.h5', 'w')
    fp.create_dataset('reproduction/x', data=opt_x)
    fp.create_dataset('reproduction/y', data=opt_y)
    fp.create_dataset('reproduction/z', data=opt_z)
    fp.close()
     
if __name__ == '__main__':
  main()