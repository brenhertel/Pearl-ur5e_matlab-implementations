import numpy as np
import matplotlib.pyplot as plt

def converging_vector_alg(traj, init=None, final=None, converge_point=0.9):
    if init == None:
        init = traj[0]
    if final == None:
        final = traj[-1]
    converge_index = converge_point * len(traj)
    THRESHOLD = 1e-10
    scale_factor = THRESHOLD**(1 / converge_index)
    init_offset = init - traj[0]
    init_offset_vector = np.ones(np.shape(traj)) * init_offset
    final_offset = final - traj[-1]
    final_offset_vector = np.ones(np.shape(traj)) * final_offset
    for i in range (len(traj)):
        init_offset_vector[i] = init_offset_vector[i] * (scale_factor**i)
        final_offset_vector[-(i + 1)] = final_offset_vector[-(i + 1)] * (scale_factor**i)
    new_traj = np.zeros(np.shape(traj))
    for i in range (len(traj)):
        new_traj[i] = traj[i] + init_offset_vector[i] + final_offset_vector[i]
    return new_traj
	
def main():
	x = np.linspace(1, 1, 1000)
	n = np.linspace(1, 1000, 1000)
	xp = converging_vector_alg(x, x[0] + 100, x[-1], 0.999)
	plt.plot(n, x, 'r')
	plt.plot(n, xp, 'g')
	plt.show()
	
if __name__ == '__main__':
  main()