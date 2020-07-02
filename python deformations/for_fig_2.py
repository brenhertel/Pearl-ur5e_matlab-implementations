import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad

def read_from_text_file(fname):
    fp = open(fname, 'r')
    line = fp.readline()
    x_data = []
    y_data = []
    while line:
        splt = line.split()
        x_data.append(float(splt[0]))
        y_data.append(float(splt[1]))
        line = fp.readline()
    x_data.reverse()
    y_data.reverse()
    return [np.array(x_data), np.array(y_data)]
    

def main():
    n_rows = 2;
    n_cols = 3;
    [s_x, s_y] = read_from_text_file('s_demo.txt')
    [a_x, a_y] = read_from_text_file('alpha_demo.txt')
    
    offset = 25
    
    [s_x_lte, s_x_ja, s_x_dmp] = pad.perform_all_deformations(s_x, initial=s_x[0])
    [s_y_lte, s_y_ja, s_y_dmp] = pad.perform_all_deformations(s_y, initial=s_y[0] - 2 * offset)
    
    [a_x_lte, a_x_ja, a_x_dmp] = pad.perform_all_deformations(a_x, end=a_x[-1] + offset)
    [a_y_lte, a_y_ja, a_y_dmp] = pad.perform_all_deformations(a_y, end=a_y[-1] - offset)
    
    plt.figure()
    plt.axis('off')
    
    plt.subplot(231)
    plt.xticks([])
    plt.yticks([])
    plt.plot(s_x, s_y, 'k', linewidth=10)
    plt.plot(s_x[0], s_y[0], 'k*', markersize=30)
    plt.plot(s_x[-1], s_y[-1], 'k.', markersize=30)
    plt.plot(s_x_lte, s_y_lte, 'g', linewidth=10)
    plt.plot(s_x_lte[0], s_y_lte[0], 'k+', markersize=30, mew=5)
    
    plt.subplot(232)
    plt.xticks([])
    plt.yticks([])
    plt.plot(s_x, s_y, 'k', linewidth=10)
    plt.plot(s_x[0], s_y[0], 'k*', markersize=30)
    plt.plot(s_x[-1], s_y[-1], 'k.', markersize=30)
    plt.plot(s_x_ja, s_y_ja, 'r', linewidth=10)
    plt.plot(s_x_ja[0], s_y_ja[0], 'k+', markersize=30, mew=5)
    
    plt.subplot(233)
    plt.xticks([])
    plt.yticks([])
    plt.plot(s_x, s_y, 'k', linewidth=10)
    plt.plot(s_x[0], s_y[0], 'k*', markersize=30)
    plt.plot(s_x[-1], s_y[-1], 'k.', markersize=30)
    plt.plot(s_x_dmp, s_y_dmp, 'b', linewidth=10)
    plt.plot(s_x_dmp[0], s_y_dmp[0], 'k+', markersize=30, mew=5)
    
    plt.subplot(234)
    plt.xticks([])
    plt.yticks([])
    plt.xlabel('LTE', fontsize=40)
    plt.plot(a_x, a_y, 'k', linewidth=10)
    plt.plot(a_x[0], a_y[0], 'k*', markersize=30)
    plt.plot(a_x[-1], a_y[-1], 'k.', markersize=30)
    plt.plot(a_x_lte, a_y_lte, 'g', linewidth=10)
    plt.plot(a_x_lte[-1], a_y_lte[-1], 'k+', markersize=30, mew=5)
    
    plt.subplot(235)
    plt.xticks([])
    plt.yticks([])
    plt.xlabel('JA', fontsize=40)
    plt.plot(a_x, a_y, 'k', linewidth=10)
    plt.plot(a_x[0], a_y[0], 'k*', markersize=30)
    plt.plot(a_x[-1], a_y[-1], 'k.', markersize=30)
    plt.plot(a_x_ja, a_y_ja, 'r', linewidth=10)
    plt.plot(a_x_ja[-1], a_y_ja[-1], 'k+', markersize=30, mew=5)
    
    plt.subplot(236)
    plt.xticks([])
    plt.yticks([])
    plt.xlabel('DMP', fontsize=40)
    plt.plot(a_x, a_y, 'k', linewidth=10)
    plt.plot(a_x[0], a_y[0], 'k*', markersize=30)
    plt.plot(a_x[-1], a_y[-1], 'k.', markersize=30)
    plt.plot(a_x_dmp, a_y_dmp, 'b', linewidth=10)
    plt.plot(a_x_dmp[-1], a_y_dmp[-1], 'k+', markersize=30, mew=5)
    
    plt.show()

if __name__ == '__main__':
    main()