import ja
import numpy as np
from scipy import optimize
import similaritymeasures
import h5py
import matplotlib.pyplot as plt

global glob_traj

def downsample_1d(traj, n=100):
    npts = np.linspace(0, len(traj) - 1, n)
    out = np.zeros((n))
    for i in range (n):
        out[i] = traj[int(npts[i])]
    print(out)
    return out

def optimize_lambda_x(x):
    global glob_traj
    new_traj = ja.perform_ja_improved(glob_traj, lmbda=x)
    fd = similaritymeasures.frechet_dist(new_traj, glob_traj)
    return fd

def opt_lambda_traj_1d(in_data, min_lambda=0.0, max_lambda=-1.0):
    if (max_lambda < 0.0):
        max_lambda = np.size(in_data) / 10.0;
    in_data = downsample_1d(in_data)
    global glob_traj
    glob_traj = in_data
    result_x = optimize.fminbound(optimize_lambda_x, min_lambda, max_lambda, disp=3)
    return result_x
    
#in-file testing
def main1():
    global glob_traj
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
    max_lambda = 200.0;
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
    result_x = opt_lambda_traj_1d(x_data)
    result_y = opt_lambda_traj_1d(y_data)
    #result = max(result_x, result_y)
    new_traj_x = ja.perform_ja_improved(x_data, initial=(-0.5 * x_data[0]), lmbda=result_x)
    new_traj_y = ja.perform_ja_improved(y_data, initial=(2.5 * y_data[0]), lmbda=result_y)
    def_traj_x = ja.perform_ja_improved(x_data, initial=(-0.5 * x_data[0]))
    def_traj_y = ja.perform_ja_improved(y_data, initial=(2.5 * y_data[0]))
    plt.plot(x_data, y_data, 'b')
    plt.plot(new_traj_x, new_traj_y, 'r')
    plt.plot(def_traj_x, def_traj_y, 'g')
    plt.show()
    

if __name__ == '__main__':
    main2()