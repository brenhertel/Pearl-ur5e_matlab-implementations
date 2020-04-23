import mlfd
import numpy as np
import h5py
import preprocess
import ja
import lte
import dmp
import os

#lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
#             'heee','JShape','JShape_2','Khamesh','Leaf_1', \
#             'Leaf_2','Line','LShape','NShape','PShape', \
#             'RShape','Saeghe','Sharpc','Sine','Snake', \
#             'Spoon','Sshape','Trapezoid','Worm','WShape', \
#             'Zshape','Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4']

lasa_names = ['RShape']

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
    for i in range (len(lasa_names)):
        metric_name = 'Endpoint convergence again'
        plt_fpath = 'mlfd_metrics/' + metric_name + '_' + lasa_names[i] + '/'
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
        my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
        my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
        #my_mlfd.add_metric(mlfd.my_curve_length2, type='shape', name=metric_name, weight=1.0, is_dissim=True)
        #my_mlfd.add_metric(mlfd.total_distance_comp2, type='length', name=metric_name, weight=2.0, is_dissim=True)
        my_mlfd.add_metric(mlfd.my_endpoint_convergence2, type='endpoint', name=metric_name, weight=1.0, is_dissim=True)
        #my_mlfd.add_metric(mlfd.my_fd2, type='geometry', name=metric_name, weight=2.0, is_dissim=True)
        my_mlfd.create_grid(9, [10, 10])
        my_mlfd.deform_traj(plot=False)
        my_mlfd.calc_metrics(d_sample=True)
        my_mlfd.save_results(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
        #my_mlfd.read_from_h5(plt_fpath + metric_name + '_' + lasa_names[i] + '.h5')
        my_mlfd.set_up_classifier()
        my_mlfd.svm_region_contour(filepath=plt_fpath)
        my_mlfd.generate_svm_region(filepath=plt_fpath)
        #my_mlfd.reproduce_at_point(np.array([[x[0][0] + 5, y[0][0] - 5]]), plot=True)
     
if __name__ == '__main__':
  main()