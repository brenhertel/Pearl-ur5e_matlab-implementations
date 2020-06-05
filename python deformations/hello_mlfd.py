import h5py
import mlfd
import numpy as np
import matplotlib.pyplot as plt
import ja
import lte
import dmp
import os


def main():
	#get data
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    y_data = hello.get('resampled_y')
    x_data = np.array(x_data)
    y_data = np.array(y_data)
    hf.close()
	#create folder to store data
    plt_fpath = 'hello_testing/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
	#set up mlfd process
    my_mlfd = mlfd.mlfd()
    my_mlfd.add_traj_dimension(x_data, 'x')
    my_mlfd.add_traj_dimension(y_data, 'y')
    my_mlfd.default_process()
    #my_mlfd.get_deform_grid_2d()
    my_mlfd.save_results(plt_fpath + 'hello_LTE_FJA_cc' + '.h5')
    #my_mlfd.add_metric(mlfd.sum_of_dists, type='Converge', name='Converge', weight=1.0, is_dissim=True)
    #my_mlfd.create_grid()
    #my_mlfd.read_from_h5(plt_fpath + 'hello_LTE_FJA_defs' + '.h5')
    my_mlfd.set_up_classifier()
    my_mlfd.svm_region_contour(filepath=plt_fpath)
    my_mlfd.generate_svm_region(filepath=plt_fpath)
    my_mlfd.reproduce_at_point(np.array([[x_data[0][0] - 1, y_data[0][0] - 1]]), plot=True)
    my_mlfd.plot_strongest_gradients_thresholded(mode='show')
		
if __name__ == '__main__':
  main()