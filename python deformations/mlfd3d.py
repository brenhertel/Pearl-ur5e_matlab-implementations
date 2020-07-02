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
    filename = 'bad preprocessed 3d_demo.h5'
    #open the file
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    demo = hf.get('demo1')
    tf_info = demo.get('tf_info')
    js_info = demo.get('joint_state_info')
    pos_rot_data = tf_info.get('pos_rot_data')
    pos_rot_data = np.array(pos_rot_data)
    js_data = js_info.get('joint_positions')
    js_data = np.array(js_data)
    #close out file
    hf.close()
    #n = 1000
    #x = preprocess.preprocess_1d(-pos_rot_data[0], n)
    #y = preprocess.preprocess_1d(-pos_rot_data[1], n)
    #z = preprocess.preprocess_1d(pos_rot_data[2], n)
    x = mlfd.downsample_1d(-pos_rot_data[0])
    y = mlfd.downsample_1d(-pos_rot_data[1])
    z = mlfd.downsample_1d(pos_rot_data[2])
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    #ax.plot(x, y, z, 'k', linewidth=10)
    #ax.plot([x[0]], [y[0]], [z[0]],  'k*', markersize=40)
    #ax.plot([x[-1]], [y[-1]], [z[-1]],  'k.', markersize=40)
    #plt.axis('off')
    #plt.show()
    #print(x)
    my_mlfd = mlfd.mlfd()
    my_mlfd.add_traj_dimension(x, 'x')
    my_mlfd.add_traj_dimension(y, 'y')
    my_mlfd.add_traj_dimension(z, 'z')
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    my_mlfd.add_metric(similaritymeasures.frechet_dist, name='Frechet', is_dissim=True)
    ##my_mlfd.add_sim_metric(mlfd.my_hd3, name='Haussdorf', is_dissim=True)
    #my_mlfd.use_default_metrics()
    #my_mlfd.create_grid()
    #my_mlfd.deform_traj(plot=False)
    #my_mlfd.calc_metrics(d_sample=False)
    #my_mlfd.save_results('3d_mlfd_data_test_for_paper.h5')
    my_mlfd.read_from_h5('3d_mlfd_data_test_for_paper.h5')
    #my_mlfd.show_3d_similarity(mode='show')
    #my_mlfd.set_up_classifier()
    #my_mlfd.vm_region_contour(mode='show')
    #my_mlfd.plot_svm_contour_3d(mode='show')
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