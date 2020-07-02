### test different DMP implementations ###
import numpy as np
import matplotlib.pyplot as plt
import perform_new_dmp
import h5py

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'C:/Users/BH/Documents/GitHub/Pearl-ur5e_matlab-implementations/python deformations')

import perform_all_deformations as pad

def get_lasa_traj1(shape_name):
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '../../h5 files/lasa_dataset.h5'
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
    #samples = 1000
    #x_data = np.linspace(0, 1, samples)
    #y_data = np.sin(4 * x_data) - 0.1 * np.cos(0.1 * x_data)
    
    [x_data, y_data] = get_lasa_traj1('heee')
    
    offset = 10
    
    
    [x_lte, x_ja, x_dmp] = pad.perform_all_deformations(x_data, x_data[0])
    [y_lte, y_ja, y_dmp] = pad.perform_all_deformations(y_data, y_data[0] - offset)
    
    plt.figure()
    demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    lte_line, = plt.plot(x_lte, y_lte, 'g', linewidth=5, label='LTE')
    plt.legend((demo_line, lte_line), ('Demonstration', 'LTE'))
    plt.show()
    
    plt.figure()
    demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    ja_line, = plt.plot(x_ja, y_ja, 'r', linewidth=5, label='JA')
    plt.legend((demo_line, ja_line), ('Demonstration', 'JA'))
    plt.show()
    
    #plt.figure()
    #demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    #init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    #end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    #dmp_line, = plt.plot(x_dmp, y_dmp, 'b', linewidth=5, label='pydmps')
    #plt.legend((demo_line, dmp_line), ('Demonstration', 'pydmps'))
    #plt.show()
    
    xt = np.transpose(x_data)
    yt = np.transpose(y_data)
    x_adapt = perform_new_dmp.perform_new_dmp_adapted(xt[0], xt[0][0], use_improved=True)
    y_adapt = perform_new_dmp.perform_new_dmp_adapted(yt[0], yt[0][0] - offset, use_improved=True)
    
    plt.figure()
    demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    new_dmp_line = plt.plot(x_adapt, y_adapt, 'm', linewidth=5)
    plt.legend((demo_line, new_dmp_line), ('Demonstration', 'new dmps'))
    plt.show()
    
    

if __name__ == '__main__':
    main()