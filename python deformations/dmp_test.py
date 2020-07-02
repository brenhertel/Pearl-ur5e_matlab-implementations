### test different DMP implementations ###
import numpy as np
import matplotlib.pyplot as plt
import dmp as pydmps
#from dmp_pp-master/dmp_cartesian import DMPs_cartesian as dmp_pp
from dmpling.dmp import DMP as dmpling

def main():
    samples = 1000
    x_data = np.linspace(0, 1, samples)
    y_data = np.sin(4 * x_data) - 0.1 * np.cos(0.1 * x_data)
    
    plt.figure()
    demo_line, = plt.plot(x_data, y_data, 'k', linewidth=10, label='Demonstration')
    init_pnt = plt.plot(x_data[0], y_data[0], 'k*', markersize=30, label='Initial Point')
    end_pnt = plt.plot(x_data[-1], y_data[-1], 'k.', markersize=30, label='Endpoint')
    
    x_pydmps = pydmps.perform_dmp_improved(x_data, initial=x_data[0] - 0.25)
    y_pydmps = pydmps.perform_dmp_improved(y_data, initial=y_data[0] - 0.25)
    
    #dmp_x = dmp_pp(dt=0.001, n_dmps=1, n_bfs=200, x_0=x_data[0] - 0.25)
    #dmp_x.imitate_path(x_data)
    #x_dmp_pp, _, _, _ = dmp_x.rollout()
    #dmp_y = dmp_pp(dt=0.001, n_dmps=1, n_bfs=200, x_0=y_data[0] - 0.25)
    #dmp_y.imitate_path(y_data)
    #y_dmp_pp, _, _, _ = dmp_y.rollout()
    
    dmpling_x = dmpling(1, dt=0.001, n_bfs=200)
    dmpling_x.fit(x_data)
    dmpling_y = dmpling(1, dt=0.001, n_bfs=200)
    dmpling_y.fit(y_data)
    dmpling_x_track = np.zeros(dmpling_x.cs.N)
    dmpling_y_track = np.zeros(dmpling_y.cs.N)
    dmpling_x.y0 = x_data[0] - 0.25
    dmpling_y.y0 = y_data[0] - 0.25
    for i in range (len(dmpling_x_track)):
        dmpling_x_track[i], _, _, _ = dmpling_x.step(start=x_data[0] - 0.25)
        dmpling_y_track[i], _, _, _ = dmpling_y.step(start=y_data[0] - 0.25)
    
    pydmps_line, = plt.plot(x_pydmps, y_pydmps, 'g', linewidth=5, label='pydmps')
    #dmp_pp_line, = plt.plot(x_dmp_pp, y_dmp_pp, 'r', linewidth=5, label='dmp_pp')
    dmpling_line, = plt.plot(dmpling_x_track, dmpling_y_track, 'b', linewidth=5, label='dmpling')
    
    #plt.axis('off')
    plt.legend((demo_line, pydmps_line, dmpling_line), ('Demonstration', 'pydmps', 'dmpling'), fontsize=25)
    plt.show()

if __name__ == '__main__':
    main()