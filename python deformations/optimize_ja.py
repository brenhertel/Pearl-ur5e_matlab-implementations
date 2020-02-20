import ja
import numpy as np
from scipy import optimize
import similaritymeasures
import h5py
import matplotlib.pyplot as plt
#from scipy.spatial.distance import directed_hausdorff

global traj

def ja_lambda(lmbda):
    global traj
    new_traj = ja.perform_ja(traj, given_points, l=lmbda)
    fd = similaritymeasures.frechet_dist(new_traj, traj)
    return fd

def optimize(traj_x):
    global traj
    traj = traj_x
    min_lambda = 0
    max_lambda = 250 #?
    #x_lambda = optimize.fmin(ja_lambda, min_lambda, max_lambda, disp=3)
    x_lambda = optimize.fminbound(test_func, min_lambda, max_lambda)
    return x_lambda
    
def test_func(x):
    return x**2
    
#in-file testing
def main():
    filename = '../h5 files/hello2.h5'
    hf = h5py.File(filename, 'r')
    hello = hf.get('hello')
    x_data = hello.get('resampled_x')
    x_data = np.array(x_data)
    hf.close()
    opt_lambda = optimize(x_data)
    opt_traj = ja.perform_ja(x_data, l=opt_lambda)
    plt.plot(x_data)
    plt.plot(opt_traj)
    plt.show()
    print(opt_lambda)
    

if __name__ == '__main__':
    main()