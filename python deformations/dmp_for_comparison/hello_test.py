import h5py
import numpy as np
import perform_new_dmp as pnd
import matplotlib.pyplot as plt
import os

D = [250., 500., 1000., 2000.]
K = [5., 10., 20., 40., 80.]

def main():
    #get data
    filename = '../../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    y_data = hello.get('resampled_y')
    x_data = np.array(x_data)
    y_data = np.array(y_data)
    hf.close()
    plt_fpath = 'hello_param_testing/'
    try:
        os.makedirs(plt_fpath)
    except OSError:
        print ("Creation of the directory %s failed" % plt_fpath)
    else:
        print ("Successfully created the directory %s" % plt_fpath)
        
    for i in range(len(K)):
        for j in range(len(D)):
            x_dmp = pnd.perform_new_dmp_adapted(x_data, k=K[i], d=D[j])
            y_dmp = pnd.perform_new_dmp_adapted(y_data, k=K[i], d=D[j])
            plt.figure()
            plt.plot(x_data, y_data, 'k', linewidth=5)
            plt.plot(x_data[0], y_data[0], 'k*', markersize=20)
            plt.plot(x_data[-1], y_data[-1], 'k.', markersize=20)
            plt.plot(x_dmp, y_dmp, 'b', linewidth=5)
            title = 'K = ' + str(K[i]) + ', D = ' + str(D[j])
            plt.title(title)
            plt.savefig(plt_fpath + title + '.png')
    
if __name__ == '__main__':
  main()