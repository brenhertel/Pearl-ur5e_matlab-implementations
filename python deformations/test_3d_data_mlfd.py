import mlfd
import numpy as np
import h5py
import preprocess
import ja
import lte
import dmp
import similaritymeasures
import os
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

def main():

	## get data fromm h5 file ##
    skill = 'PRESSING'
    filename = '../h5 files/' + skill + '_dataset.h5'
    #open the file
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    f1 = hf.get('file1')
    d1 = f1.get('demo1')
    pos = d1.get('pos')
	
	## convert data to useful arrays ##
    pos_arr = np.array(pos)
    x = pos[0]
    y = pos[1]
    z = pos[2]
    hf.close()
	
	## plot demo ##
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    #ax.plot(x, y, z, 'k', linewidth=10)
    #ax.plot([x[0]], [y[0]], [z[0]],  'k*', markersize=40)
    #ax.plot([x[-1]], [y[-1]], [z[-1]],  'k.', markersize=40)
    #plt.axis('off')
    #plt.show()
    
    ## set up mlfd ##
    my_mlfd = mlfd.mlfd()
    my_mlfd.add_traj_dimension(x, 'x')
    my_mlfd.add_traj_dimension(y, 'y')
    my_mlfd.add_traj_dimension(z, 'z')
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    my_mlfd.add_metric(similaritymeasures.frechet_dist, name='Frechet', is_dissim=True)
    
    ## first run params ##
    #my_mlfd.create_grid()
    #my_mlfd.deform_traj()
    #my_mlfd.calc_metrics()
    
    ## save/load from file ##
    #my_mlfd.save_results(skill + '_mlfd.h5')
    my_mlfd.read_from_h5(skill + '_mlfd.h5')
    
    ## interpret results ##
    #my_mlfd.show_3d_similarity(mode='show')
    my_mlfd.set_up_classifier()
    #my_mlfd.svm_region_contour(mode='show')
    #my_mlfd.plot_svm_contour_3d(mode='show')
    my_mlfd.show_3d_similarity_optimal(mode='show')
    ##my_mlfd.plot_surfaces(mode='show')
    ##my_mlfd.plot_surfaces(filepath='3d_demo_data/')
    ##my_mlfd.plot_sim(0.5, mode='show', filepath='')
    ##print(my_mlfd._get_strongest_repro(0.5)[5][5][5][0])
    #my_mlfd.set_up_classifier()
    ##print(np.array([[x[0], y[0], z[0]]]))
    ##my_mlfd.query_svm(np.array([[x[0], y[0], z[0]]]), plot=False)
    #e = 0.01
    #my_mlfd.query_svm(np.array([[x[0] + e, y[0] + e, z[0] + e]]), plot=True)
    ##my_mlfd.query_knn(np.array([[x[0], y[0], z[0]]]), plot=False)
    ##my_mlfd.query_knn(np.array([[1.5, 1.0, 1.5]]), plot=False)
     
if __name__ == '__main__':
  main()