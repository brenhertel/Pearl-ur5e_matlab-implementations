import ja
import numpy as np
from scipy import optimize
import similaritymeasures
import h5py
import matplotlib.pyplot as plt

def test_func_x(x):
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    x_data = np.array(x_data)
    hf.close()
    new_traj = ja.perform_ja_improved(x_data, lmbda=x)
    fd = similaritymeasures.frechet_dist(new_traj, x_data)
    return fd

def test_func_y(x):
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    y_data = hello.get('resampled_y')
    y_data = np.array(y_data)
    hf.close()
    new_traj = ja.perform_ja_improved(y_data, lmbda=x)
    fd = similaritymeasures.frechet_dist(new_traj, y_data)
    return fd

#in-file testing
def main():
    min_lambda = 0.0;
    max_lambda = 250.0;
    result_x = optimize.fminbound(test_func_x, min_lambda, max_lambda, disp=3)
    result_y = optimize.fminbound(test_func_y, min_lambda, max_lambda, disp=3)
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    #x_data = np.transpose(np.array(x_data))
    x_data = np.array(x_data)
    y_data = hello.get('resampled_y')
    #y_data = np.transpose(np.array(y_data))
    y_data = np.array(y_data)
    hf.close()
    new_traj_x = ja.perform_ja_improved(x_data, lmbda=result_x)
    new_traj_y = ja.perform_ja_improved(y_data, lmbda=result_y)
    def_traj_x = ja.perform_ja_improved(x_data)
    def_traj_y = ja.perform_ja_improved(y_data)
    plt.plot(x_data, y_data, 'b')
    plt.plot(new_traj_x, new_traj_y, 'r')
    plt.plot(def_traj_x, def_traj_y, 'g')
    plt.show()
    

if __name__ == '__main__':
    main()