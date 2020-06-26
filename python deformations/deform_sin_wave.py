import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp
import perform_all_deformations as pad

def main():
    samples = 1000
    x_data = np.linspace(0, 1, samples)
    y_data = np.sin(4 * x_data) - 0.1 * np.cos(0.1 * x_data)
    plt.figure()
    demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    [x_lte, x_ja, x_dmp] = pad.perform_all_deformations(x_data, x_data[0] - 0.25)
    [y_lte, y_ja, y_dmp] = pad.perform_all_deformations(y_data, y_data[0] - 0.25)
    lte_line, = plt.plot(x_lte, y_lte, 'g', linewidth=5, label='LTE')
    ja_line, = plt.plot(x_ja, y_ja, 'r', linewidth=5, label='JA')
    dmp_line, = plt.plot(x_dmp, y_dmp, 'b', linewidth=5, label='DMP')
    #plt.axis('off')
    plt.legend((demo_line, lte_line, ja_line, dmp_line, init_pnt, end_pnt), ('Demonstration', 'LTE', 'JA', 'DMP', 'Initial Point', 'Endpoint'), fontsize=25)
    plt.show()

if __name__ == '__main__':
    main()