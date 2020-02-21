import dmp
import numpy as np
from scipy import optimize
import similaritymeasures
import h5py
import matplotlib.pyplot as plt

global glob_traj1
global glob_traj2
global glob_alpha_y

def downsample_1d(traj, n=100):
    npts = np.linspace(0, len(traj) - 1, n)
    out = np.zeros((n))
    for i in range (n):
        out[i] = traj[int(npts[i])]
    print(out)
    return out

def optimize_alpha_x(x):
    global glob_traj1
    global glob_traj2
    global glob_alpha_y
    new_traj1 = dmp.perform_dmp(glob_traj1, given_points=0, alpha_y=(np.ones(200) * x), beta_y=None)
    end_dist1 = abs(new_traj1[len(new_traj1) - 1] - glob_traj1[len(glob_traj1) - 1])
    new_traj2 = dmp.perform_dmp(glob_traj2, given_points=0, alpha_y=(np.ones(200) * x), beta_y=None)
    end_dist2 = abs(new_traj2[len(new_traj2) - 1] - glob_traj2[len(glob_traj2) - 1])
    end_dist = (end_dist1**2 + end_dist2**2)**0.5
    return end_dist

def opt_alpha_traj_2d(in_data1, in_data2, min_alpha=0.0, max_alpha=1000.0):
    in_data1 = downsample_1d(in_data1)
    in_data2 = downsample_1d(in_data2)
    global glob_traj1
    glob_traj1 = in_data1
    global glob_traj2
    glob_traj2 = in_data2
    result_x = optimize.fminbound(optimize_alpha_x, min_alpha, max_alpha, disp=3)
    return result_x

def opt_alpha_traj_1d(in_data, min_alpha=0.0, max_alpha=1000.0):
    in_data = downsample_1d(in_data)
    global glob_traj
    glob_traj = in_data
    result_x = optimize.fminbound(optimize_alpha_x, min_alpha, max_alpha, disp=3)
    return result_x
    
def fix_traj(traj):
    #transpose if necessary
  if len(np.shape(traj)) > 1:
    if np.shape(traj)[0] > np.shape(traj)[1]:
      traj = np.transpose(traj)
  traj = np.reshape(traj, (1, max(np.shape(traj))))
  return traj
    
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
    min_alpha = 0.0;
    max_alpha = 1000.0;
    result_x = optimize.fminbound(optimize_lambda_x, min_alpha, max_alpha, disp=3)
    result_y = optimize.fminbound(optimize_lambda_y, min_alpha, max_alpha, disp=3)
    new_traj_x = dmp.perform_dmp(x_data, given_points=0, alpha_y=(np.ones(200) * result_x), beta_y=None)
    new_traj_y = dmp.perform_dmp(y_data, given_points=0, alpha_y=(np.ones(200) * result_y), beta_y=None)
    def_traj_x = dmp.perform_dmp(x_data, given_points=0, alpha_y=None, beta_y=None)
    def_traj_y = dmp.perform_dmp(y_data, given_points=0, alpha_y=None, beta_y=None)
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
    global glob_alpha_y
    glob_alpha_y = 61.9457
    result_x = opt_alpha_traj_1d(x_data)
    glob_alpha_y = 31.6852
    result_y = opt_alpha_traj_1d(y_data)
    xn_data = fix_traj(x_data)
    yn_data = fix_traj(y_data)
    new_traj_x = dmp.perform_dmp(xn_data, given_points=0, alpha_y=61.9457, beta_y=result_x)
    new_traj_y = dmp.perform_dmp(yn_data, given_points=0, alpha_y=31.6852, beta_y=result_y)
    def_traj_x = dmp.perform_dmp(xn_data, given_points=0, alpha_y=None, beta_y=None)
    def_traj_y = dmp.perform_dmp(yn_data, given_points=0, alpha_y=None, beta_y=None)
    plt.plot(x_data, y_data, 'b')
    plt.plot(new_traj_x, new_traj_y, 'r')
    plt.plot(def_traj_x, def_traj_y, 'g')
    plt.show()
    
 #in-file testing
def main3():
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
    result = opt_alpha_traj_2d(x_data, y_data)
    xn_data = fix_traj(x_data)
    yn_data = fix_traj(y_data)
    new_traj_x = dmp.perform_dmp(xn_data, given_points=0, alpha_y=result, beta_y=None)
    new_traj_y = dmp.perform_dmp(yn_data, given_points=0, alpha_y=result, beta_y=None)
    def_traj_x = dmp.perform_dmp(xn_data, given_points=0, alpha_y=None, beta_y=None)
    def_traj_y = dmp.perform_dmp(yn_data, given_points=0, alpha_y=None, beta_y=None)
    plt.plot(x_data, y_data, 'b')
    plt.plot(new_traj_x, new_traj_y, 'r')
    plt.plot(def_traj_x, def_traj_y, 'g')
    plt.show()
    
if __name__ == '__main__':
    main3()