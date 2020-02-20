import ja
import numpy as np
from scipy import optimize
import similaritymeasures
import h5py
import matplotlib.pyplot as plt

global x_traj
global y_traj

def downsample(traj, n=100):
    npts = np.linspace(0, len(traj) - 1, n)
    out = np.zeros((np.shape(traj)))
    for i in range (n):
        out[i] = traj[int(npts[i])]
    return out

def optimize_lambda_x(x):
    global x_traj
    new_traj = ja.perform_ja_improved(x_traj, lmbda=x)
    fd = similaritymeasures.frechet_dist(new_traj, x_traj)
    return fd

def optimize_lambda_y(x):
    global y_traj
    new_traj = ja.perform_ja_improved(y_traj, lmbda=x)
    fd = similaritymeasures.frechet_dist(new_traj, y_traj)
    return fd

def opt_lambda_traj_1d(x_data, min_lambda = 0.0, max_lambda = 250.0):
    x_data = downsample(x_data)
    global x_traj
    x_traj = x_data
    result_x = optimize.fminbound(optimize_lambda_x, min_lambda, max_lambda, disp=3)
    return result_x
    
#in-file testing
def main1():
    global x_traj
    global y_traj
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
    x_traj = x_data
    y_traj = y_data
    min_lambda = 0.0;
    max_lambda = 250.0;
    result_x = optimize.fminbound(optimize_lambda_x, min_lambda, max_lambda, disp=3)
    result_y = optimize.fminbound(optimize_lambda_y, min_lambda, max_lambda, disp=3)
    new_traj_x = ja.perform_ja_improved(x_data, lmbda=result_x)
    new_traj_y = ja.perform_ja_improved(y_data, lmbda=result_y)
    def_traj_x = ja.perform_ja_improved(x_data)
    def_traj_y = ja.perform_ja_improved(y_data)
    plt.plot(x_data, y_data, 'b')
    plt.plot(new_traj_x, new_traj_y, 'r')
    plt.plot(def_traj_x, def_traj_y, 'g')
    plt.show()
    
 #in-file testing
def main2():
    global x_traj
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
    result_x = opt_lambda_traj_1d
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