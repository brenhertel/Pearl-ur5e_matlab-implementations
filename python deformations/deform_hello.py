#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad

def main():
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    y_data = hello.get('resampled_y')
    x_data = np.array(x_data)
    y_data = np.array(y_data)
    hf.close()
    #print(x_data[-1][0])
    plt.figure()
    plt.plot(x_data, y_data, 'k', linewidth=5)
    [x_lte, x_ja, x_dmp] = pad.perform_all_deformations(x_data, x_data[0][0] - 1.5, x_data[-1][0] - 1.5)
    [y_lte, y_ja, y_dmp] = pad.perform_all_deformations(y_data, y_data[0][0] - 1.5, y_data[-1][0] - 1.5)
    plt.plot(x_lte, y_lte, 'g')
    plt.plot(x_ja, y_ja, 'r')
    plt.plot(x_dmp, y_dmp, 'b')
    plt.show()

if __name__ == '__main__':
    main()