#!/usr/bin/env python

#This is the interface for the pydmps code, originally found here: https://github.com/studywolf/pydmps

import numpy as np
import pydmps
import pydmps.dmp_discrete
import matplotlib.pyplot as plt

#to be called from another file
#uses code in the pydmps folder
def perform_dmp(traj, given_points=0):
  dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=1, n_bfs=200, ay=np.ones(2) * 10.0)
  y_track = []
  dy_track = []
  ddy_track = []
  dmp.imitate_path(y_des=traj, plot=False)
  if given_points != 0:
    dmp.y0[0] = given_points[0]
    dmp.goal[0] = given_points[1]
  y_track, dy_track, ddy_track = dmp.rollout()
  return y_track

#in-file testing
def main():
  y_des = np.linspace(1, 25, 100)
  dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=1, n_bfs=100)
  y_track = []
  dy_track = []
  ddy_track = []
  dmp.imitate_path(y_des=y_des, plot=False)
  y_track, dy_track, ddy_track = dmp.rollout()
  print(y_track)

if __name__ == '__main__':
  main()