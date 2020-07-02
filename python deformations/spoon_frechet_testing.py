#frechet saeghe -> generates good regions
#generate heatmaps
#generate SVM region
#generate thresholded SVM region?

import mlfd_endpoint
import numpy as np
import h5py
import preprocess
import ja
import lte
import dmp
import os
import matplotlib.pyplot as plt
import similaritymeasures

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'C:/Users/BH/Documents/GitHub/Pearl-ur5e_matlab-implementations/python deformations/dmp_for_comparison/')

import perform_new_dmp as pnd

lasa_names = ['LShape']


metric_funx = [similaritymeasures.frechet_dist]

metric_names = ['Frechet']




def get_lasa_traj1(shape_name):
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    shape = hf.get(shape_name)
    demo = shape.get('demo1')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    #close out file
    hf.close()
    return [x_data, y_data]

def main():
    global lasa_names
    global metric_names
    global metric_funx
    for i in range (len(lasa_names)):
        for j in range(len(metric_names)):
            metric_name = metric_names[j]
            plt_fpath = 'FOR_PAPER/' + metric_name + '_' + lasa_names[i] + '/'
            try:
                os.makedirs(plt_fpath)
            except OSError:
                print ("Creation of the directory %s failed" % plt_fpath)
            else:
                print ("Successfully created the directory %s" % plt_fpath)
            [x, y] = get_lasa_traj1(lasa_names[i])
            my_mlfd = mlfd_endpoint.mlfd()
            my_mlfd.add_traj_dimension(x, 'x')
            my_mlfd.add_traj_dimension(y, 'y')
            my_mlfd.add_deform_alg(ja.perform_ja_improved, 'FJA')
            my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
            my_mlfd.add_deform_alg(pnd.perform_new_dmp_adapted, 'DMP')
            my_mlfd.add_metric(metric_funx[j], type='Frechet', name=metric_name, weight=1.0, is_dissim=True)
            my_mlfd.create_grid()
            my_mlfd.deform_traj(plot=False)
            my_mlfd.get_deform_grid_2d(mode='save', filepath=plt_fpath)
            my_mlfd.calc_metrics(d_sample=True)
            #my_mlfd.save_results(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
            #my_mlfd.read_from_h5(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
            my_mlfd.set_up_classifier()
            my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath)
            my_mlfd.generate_svm_region(filepath=plt_fpath)
            #my_mlfd.reproduce_at_point(np.array([[x[0][0] + 5, y[0][0] - 5]]), plot=True)
            #my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
            #my_mlfd.plot_sim_hmap(mode='save', filepath=plt_fpath)
            
            my_mlfd.reproduce_optimal_at_point(np.array([[x[-1][0] + 3.5, y[-1][0] - 2.5]]), plot=True, mode='save', filepath=plt_fpath)
            my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath, plot_point=[x[-1][0] + 3.5, y[-1][0] - 2.5])
            
            my_mlfd.reproduce_optimal_at_point(np.array([[x[-1][0] + 4.5, y[-1][0] + 4.5]]), plot=True, mode='save', filepath=plt_fpath)
            my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath, plot_point=[x[-1][0] + 4.5, y[-1][0] + 4.5])
            
            my_mlfd.reproduce_optimal_at_point(np.array([[x[-1][0] - 4.5, y[-1][0] - 4.5]]), plot=True, mode='save', filepath=plt_fpath)
            my_mlfd.svm_region_contour(mode='save', filepath=plt_fpath, plot_point=[x[-1][0] - 4.5, y[-1][0] - 4.5])
     
if __name__ == '__main__':
  main()