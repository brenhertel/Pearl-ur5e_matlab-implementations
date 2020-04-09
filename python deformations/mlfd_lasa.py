import mlfd
import numpy as np
import h5py
import preprocess
import ja
import lte
import dmp
import os

lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
              'heee','JShape','JShape_2','Khamesh','Leaf_1', \
              'Leaf_2','Line','LShape','NShape','PShape', \
              'RShape','Saeghe','Sharpc','Sine','Snake', \
              'Spoon','Sshape','Trapezoid','Worm','WShape', \
              'Zshape','Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4']

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
        plt_fpath = 'lasa_mlfd/' + lasa_names[i] + '/'
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
        my_mlfd.add_deform_alg(ja.perform_ja_improved, 'JA')
        my_mlfd.add_deform_alg(lte.perform_lte_improved, 'LTE')
        my_mlfd.add_deform_alg(dmp.perform_dmp_improved, 'DMP')
        my_mlfd.use_default_metrics()
        my_mlfd.create_grid(9, [10, 10])
        my_mlfd.deform_traj(plot=False)
        my_mlfd.calc_metrics(d_sample=False)
        my_mlfd.save_results(plt_fpath + 'mlfd_lasa_' + lasa_names[i] + '.h5')
        #my_mlfd.read_from_h5('3d_mlfd_data_test_new_n.h5')
        my_mlfd.set_up_classifier()
        my_mlfd.svm_region_contour(filepath=plt_fpath)
        my_mlfd.generate_svm_region(filepath=plt_fpath)
     
if __name__ == '__main__':
  main()