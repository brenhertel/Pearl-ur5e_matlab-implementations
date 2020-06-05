import mlfd
import numpy as np
import h5py
import preprocess
import ja
import lte
import dmp
import os
import matplotlib.pyplot as plt
import converging_vector_alg
import preserving_vector_alg

lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
             'heee','JShape','JShape_2','Khamesh','Leaf_1', \
             'Leaf_2','Line','LShape','NShape','PShape', \
             'RShape','Saeghe','Sharpc','Sine','Snake', \
             'Spoon','Sshape','Trapezoid','Worm','WShape', \
             'Zshape','Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4']


#metric_funx = [mlfd.my_hd2, mlfd.my_fd2, mlfd.my_dtw2, mlfd.my_curvature_conservation2, \
#                mlfd.my_crv2, mlfd.my_jerk2, mlfd.my_endpoint_convergence2, mlfd.my_curve_length2, \
#                mlfd.my_pcm2, mlfd.my_area_eval2, mlfd.sim_measure_dtw2, mlfd.total_distance_comp2]
#
#metric_names = ['Haussdorff', 'Frechet', 'DTW', 'Curvature_Conservation', \
#                'Curvature_Conservation2', 'Jerk_Evaluation', 'Endpoint_Convergence', 'Curve_Length', \
#                'PCM', 'Area', 'DTW2', 'Curve_Length2']

#metric_funx = [mlfd.my_hd2, mlfd.my_fd2,  \
#                mlfd.my_crv2, mlfd.my_endpoint_convergence2, mlfd.my_curve_length2, \
#                mlfd.my_pcm2, mlfd.my_area_eval2, mlfd.sim_measure_dtw2, mlfd.total_distance_comp2]
#
#metric_names = ['Haussdorff', 'Frechet', \
#                'Curvature_Conservation2', 'Endpoint_Convergence', 'Curve_Length', \
#                'PCM', 'Area', 'DTW', 'Curve_Length2']

def_metric_funx = [mlfd.sum_of_dists, mlfd.curvature_comparison]
def_metric_names = ['sum_of_dists', 'curvature_comparison']

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
    global def_metric_names
    global def_metric_funx
    #for i in range (len(lasa_names)):
    #    for j in range(len(def_metric_names)):
    for i in range (len(lasa_names)):
        for j in range(len(def_metric_names)):
            metric_name = def_metric_names[j]
            plt_fpath = 'mlfd_metrics/LTE_FJA_tests/' + metric_name + '_' + lasa_names[i] + '/'
            try:
                os.makedirs(plt_fpath)
            except OSError:
                print ("Creation of the directory %s failed" % plt_fpath)
            else:
                print ("Successfully created the directory %s" % plt_fpath)
            [x, y] = get_lasa_traj1(lasa_names[i])
            my_mlfd = mlfd.mlfd()
            my_mlfd.add_traj_dimension(x, 'x')
            my_mlfd.add_traj_dimension(y, 'y')
            my_mlfd.add_deform_alg(ja.perform_ja_improved, 'FJA')
            #my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
            my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
            #my_mlfd.add_deform_alg(converging_vector_alg.converging_vector_alg, 'CVA')
            #my_mlfd.add_deform_alg(preserving_vector_alg.preserving_vector_alg, 'PVA')
            my_mlfd.add_metric(def_metric_funx[j], type='custom', name=metric_name, weight=1.0, is_dissim=True)
            my_mlfd.create_grid(9, [10, 10])
            my_mlfd.deform_traj(plot=False)
            if (j == 0):
                my_mlfd.get_deform_grid_2d(mode='save', filepath=plt_fpath)
                plt.plot(x, y, 'k')
                plt.savefig(plt_fpath + lasa_names[i] + '_Original' + '.png')
                plt.close('all')
            my_mlfd.calc_metrics(d_sample=True)
            my_mlfd.save_results(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
            #my_mlfd.read_from_h5(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
            #my_mlfd.set_up_classifier()
            #my_mlfd.svm_region_contour(filepath=plt_fpath)
            #my_mlfd.generate_svm_region(filepath=plt_fpath)
            #my_mlfd.reproduce_at_point(np.array([[x[0][0] + 5, y[0][0] - 5]]), plot=True)
            my_mlfd.plot_strongest_gradients(mode='save', filepath=plt_fpath)
     
if __name__ == '__main__':
  main()