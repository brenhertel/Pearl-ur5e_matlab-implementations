import h5py
import mlfd
import numpy as np
import matplotlib.pyplot as plt
import ja
import lte
import dmp
import similaritymeasures
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
    plt_fpath = 'FOR_PAPER/Frechet_hello/'
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
    my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
    my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
    my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
    my_mlfd.add_metric(similaritymeasures.frechet_dist, type='Frechet', name='Frechet', weight=1.0, is_dissim=True)
    #my_mlfd.create_grid()
    #my_mlfd.deform_traj(plot=False)
    #my_mlfd.get_deform_grid_2d(mode='save', filepath=plt_fpath)
    #my_mlfd.calc_metrics(d_sample=True)
    #my_mlfd.save_results(plt_fpath + 'Frechet_hello.h5')
    my_mlfd.read_from_h5(plt_fpath + 'Frechet_hello.h5')
    my_mlfd.set_up_classifier()
    my_mlfd.svm_region_contour(filepath=plt_fpath)
    my_mlfd.generate_svm_region(filepath=plt_fpath)
    #my_mlfd.reproduce_at_point(np.array([[x[0][0] + 5, y[0][0] - 5]]), plot=True)
    #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
    #my_mlfd.plot_sim_hmap(mode='save', filepath=plt_fpath)
    offset = 0.35
    #my_mlfd.reproduce_optimal_at_point(np.array([[x_data[0][0] + offset - 0.5, y_data[0][0] + offset]]), plot=True, mode='save', filepath=plt_fpath)
    #my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath, plot_point=[x_data[0][0] + offset - 0.5, y_data[0][0] + offset])
    my_mlfd.reproduce_optimal_at_point(np.array([[x_data[0][0] + offset / 2, y_data[0][0] - 0]]), plot=True, mode='save', filepath=plt_fpath)
    my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath, plot_point=[x_data[0][0] + offset / 2, y_data[0][0] - 0])
		
if __name__ == '__main__':
  main()