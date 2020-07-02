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

#skills = ['PUSHING', 'PRESSING', 'REACHING', 'WRITING']
skills = ['REACHING', 'WRITING']

def main():
    global skills
    for i in range(len(skills)):
        ## get data fromm h5 file ##
        skill = skills[i]
        filename = '../h5 files/' + skill + '_dataset.h5'
        #open the file
        hf = h5py.File(filename, 'r')
        print(hf)
        #navigate to necessary data and store in numpy arrays
        for fnum in range(len(hf.keys())):
            fname = 'file' + str(fnum + 1)
            print(fname)
            f = hf.get(fname)
            print(f)
            for dnum in range(len(f.keys())):
                dname = 'demo' + str(dnum + 1) 
                print(dname)
                d = f.get(dname)
                print(d)
                pos = d.get('pos')
                
                ## convert data to useful arrays ##
                pos_arr = np.array(pos)
                n_dsample = 100;
                x = mlfd.downsample_1d(pos[0], n_dsample)
                y = mlfd.downsample_1d(pos[1], n_dsample)
                z = mlfd.downsample_1d(pos[2], n_dsample)
                
                ## set up save data filepath ##
                plt_fpath = '3d_testing/' + skill + '/' + fname + '/' + dname + '/'
                try:
                    os.makedirs(plt_fpath)
                except OSError:
                    print ("Creation of the directory %s failed" % plt_fpath)
                else:
                    print ("Successfully created the directory %s" % plt_fpath)
                
                ## plot demo ##
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
                my_mlfd.create_grid()
                my_mlfd.deform_traj()
                plt.close('all')
                my_mlfd.calc_metrics(d_sample=False)
                
                ## save/load from file ##
                my_mlfd.save_results(plt_fpath + skill + '_mlfd.h5')
                #my_mlfd.read_from_h5(skill + '_mlfd.h5')
                
                ## interpret results ##
                my_mlfd.set_up_classifier()
                my_mlfd.generate_svm_region(mode='save', filepath=plt_fpath)
                my_mlfd.get_image_slices(mode='save', filepath=plt_fpath)
                my_mlfd.show_3d_similarity_optimal(mode='save', filepath=plt_fpath)
        hf.close()
     
if __name__ == '__main__':
  main()