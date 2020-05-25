import numpy as np
import matplotlib.pyplot as plt

##I have no idea if this will work

def lin_sampling_funx(length):
	return np.linspace(1, 0, length), np.linspace(0, 1, length)

def sampling_function(length):
	return lin_sampling_funx(length)

def preserving_vector_alg(traj, init=None, final=None):
    if init == None:
        init = traj[0]
    if final == None:
        final = traj[-1]
        
    init_point_importance, final_point_importance = sampling_function(len(traj))
		
    init_offset = init - traj[0]
    init_offset_vector = np.ones(np.shape(traj)) * init_offset
    final_offset = final - traj[-1]
    final_offset_vector = np.ones(np.shape(traj)) * final_offset
    new_traj = np.zeros(np.shape(traj))
    for i in range (len(traj)):
        new_traj[i] = traj[i] + (init_offset_vector[i] * init_point_importance[i]) + (final_offset_vector[i] * final_point_importance[i])
    return new_traj
	
def main():
	#x = np.linspace(1, 1, 1000)
	#n = np.linspace(1, 1000, 1000)
	#xp = preserving_vector_alg(x, x[0] + 100, x[-1] + 100)
	#plt.plot(n, x, 'r')
	#plt.plot(n, xp, 'g')
	#plt.show()
    x = np.linspace(-10, 10, 1000)
    y = np.power(x, 2)
    xp = preserving_vector_alg(x, -15)
    yp = preserving_vector_alg(y, y[0] - 100)
    plt.plot(x, y, 'r')
    plt.plot(xp, yp, 'g')
    plt.show()
	
if __name__ == '__main__':
  main()