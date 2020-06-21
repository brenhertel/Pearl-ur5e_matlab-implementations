#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad

def main():
    filename = '../h5 files/Straight_Ribbon_drawing_demo.h5'
    hf = h5py.File(filename, 'r')
    sr = hf.get('Straight_Ribbon')
    x_data = sr.get('x')
    y_data = sr.get('y')
    x_data = np.array(x_data)
    y_data = np.array(y_data)
    hf.close()
    #print(x_data[-1][0])
    plt.figure()
    plt.plot(x_data[0][0], y_data[0][0], 'k*', markersize=30)
    plt.plot(x_data[-1][0], y_data[-1][0], 'k.', markersize=30)
    plt.plot(x_data, y_data, 'k', linewidth=5)
    [x_lte, x_ja, x_dmp] = pad.perform_all_deformations(x_data, x_data[0][0] + 5.5, x_data[-1][0])
    [y_lte, y_ja, y_dmp] = pad.perform_all_deformations(y_data, y_data[0][0] + 3.5, y_data[-1][0])
    plt.plot(x_lte, y_lte, 'g', linewidth=4)
    plt.plot(x_ja, y_ja, 'r', linewidth=4)
    plt.plot(x_dmp, y_dmp, 'b', linewidth=4)
    plt.axis('off')
    plt.show()

if __name__ == '__main__':
    main()