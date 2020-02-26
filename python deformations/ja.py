#!/usr/bin/env python

#python (for ROS purposes) implementation of Jerk-Accuracy (JA) trajectory editing.
#Original: https://www.mathworks.com/matlabcentral/fileexchange/58403-kinematic-filtering-for-human-and-robot-trajectories
#Each JA object is used to create a deformed trajectory for any 1D given trajectory
#	Note: Theoretically this handles n-D trajectories, but hasn't been tested with n-D. Using n-D is the same as using n 1D trajectories

import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.integrate import solve_bvp
import math
import matplotlib.pyplot as plt

#use endpoint object to define the endpoints
class endpoint(object):
  def __init__(self, position, velocity=0, acceleration=0):
    #endpoints must have position, and optionally velocity and/or acceleration
    self.x = position
    self.v = velocity
    self.a = acceleration

class JA(object):
  def __init__(self, given_traj, given_points=[], given_lambda=-1.0, given_time_data=0, given_direction=1, given_method='fast'):
    #given dims x nodes, transform into nodes x dims
    self.traj = np.transpose(given_traj)
    #variables for ease
    self.nodes = np.shape(self.traj)[0]
    self.dims = np.shape(self.traj)[1]
    #set up endpoints
    self.endpoints = given_points
    if self.endpoints == []:
      for i in range (self.dims):
	#set the default endpoints as the first and last point in the given trajectory
        endpnt1 = endpoint(self.traj[0, i])
        endpnt2 = endpoint(self.traj[self.nodes - 1, i])
        self.endpoints.append(endpnt1)
        self.endpoints.append(endpnt2)
        #reshape endpoint list into 2 x dims 
        self.endpoints = np.reshape(self.endpoints, (2, self.dims))
    #setup lambda
    self.l = given_lambda
    if self.l <= 0.0:
      self.l = math.ceil(np.size(self.traj) / 20.0) #I'm not sure where the next 2 lines come from but they were in the matlab code
    self.l = self.l * (self.nodes / 250.0) * (self.dims**2 / 4.0)
    #setup time data
    self.tt = given_time_data
    if self.tt == 0:
      self.tt = np.linspace(0, 1, self.nodes)
    #setup direction & method
    self.direction = given_direction
    self.method = given_method
    
  def generateTraj(self):
    for di in range (self.dims):
      #set up interpolated cubic spline over the time data to estimate function values
      F = InterpolatedUnivariateSpline(self.tt, self.traj[:, di], k=3)
      self.f = F
      #Reference for what the endpoints are w/ comparisons to their names in the original matlab code
        #rx0 = self.endpoints[0, di].x
        #rx1 = self.endpoints[1, di].x
        #vx0 = self.endpoints[0, di].v
        #vx1 = self.endpoints[1, di].v
        #ax0 = self.endpoints[0, di].a
        #ax1 = self.endpoints[1, di].a
      #function handles
      def bc(y0, y1):
        res = np.array([y0[0] - self.endpoints[0, di].x, y0[1] - self.endpoints[0, di].v, y0[2] - self.endpoints[0, di].a, y1[0] - self.endpoints[1, di].x, y1[1] - self.endpoints[1, di].v, y1[2] - self.endpoints[1, di].a]).reshape(6)
        return res
      #MSD is different from original due to how the solve_bvp function works - same mathematical operations 
      def MSDAccuracy_DE(t, y):
        p = 6
        dydt = np.zeros(np.shape(y))
        for i in range (len(t)):
          dydt[0, i] = y[1, i]
          dydt[1, i] = y[2, i]
          dydt[2, i] = y[3, i]
          dydt[3, i] = y[4, i]
          dydt[4, i] = y[5, i]
          dydt[5, i] = self.direction * (self.l**p) * (y[0, i] - self.f(t[i]))
        return dydt
      #create guess function
      #starting and ending times & positions
      t1 = self.tt[0]
      t2 = self.tt[len(self.tt) - 1]
      x0 = self.endpoints[0, di].x - F(t1)
      xf = self.endpoints[1, di].x - F(t2)
      #derivitaves guess
      denom = (t1 - t2)**5
      #coefficients of time variable
      a0 = x0 + (t2 * (5 * (t1**4) * x0 - 5 * (t1**4) * xf) - (t1**5) * x0 + (t1**5) * xf - (t2**2) * (10 * (t1**3) * x0 - 10 * (t1**3) * xf)) / denom
      a1 = (30 * (t1**2) * (t2**2) * (x0-xf)) / denom
      a2 = -(30 * t1 * t2 * (t1 + t2) * (x0 - xf)) / denom
      a3 = (10 * (x0 - xf) * ((t1**2) + 4 * t1 * t2 + (t2**2))) / denom
      a4 = -(15 * (t1 + t2) * (x0 - xf)) / denom
      a5 = (6 * (x0 - xf)) / denom
      y = np.zeros((6, self.nodes))
      for t in range (len(self.tt)):
        y[0, t] = F(self.tt[t]) + a5 * (self.tt[t]**5) + a4 * (self.tt[t]**4) + a3 * (self.tt[t]**3) + a2 * (self.tt[t]**2) + a1 * self.tt[t] + a0
        y[1, t] = 5 * a5 * (self.tt[t]**4) + 4 * a4 * (self.tt[t]**3) + 3 * a3 * (self.tt[t]**2) + 2 * a2 * self.tt[t] + a1
        y[2, t] = 20 * a5 * (self.tt[t]**3) + 12 * a4 * (self.tt[t]**2) + 6 * a3 * self.tt[t] + 2 * a2
        y[3, t] = 60 * a5 * (self.tt[t]**2) + 24 * a4 * self.tt[t] + 6 * a3
        y[4, t] = 120 * a5 * self.tt[t] + 24 * a4
        y[5, t] = 120 * a5
      #solve the BVP 
      sol = solve_bvp(MSDAccuracy_DE, bc, self.tt, y, max_nodes=self.nodes)
      #Check if it didn't converge - would be due to bad guess
      if sol.status != 0:
        print("WARNING: sol.status is %d" % sol.status)
        print(sol.message)
      #slow indicates a recalculation of the BVP with the guess using the previously found solution
      if self.method == 'slow':
        sol = solve_bvp(MSDAccuracy_DE, bc, sol.x, sol.y, max_nodes=self.nodes)
        if sol.status != 0:
          print("WARNING: sol.status is %d" % sol.status)
          print(sol.message)
    #this implementation is only valid for 1D trajectories. If give >1D trajectory, change the return value
    return sol.y[0]

#function to be called from a different file, sets up the fixed points correctly
def generate_ja_fixed_points(positions, velocities=0, accelerations=0):
  point_arr = []
  if velocities == 0:
    velocities = np.zeros((np.shape(positions)))
  if accelerations == 0:
    accelerations = np.zeros((np.shape(positions)))
  for i in range (np.shape(positions)[1]):
    add_start_point = endpoint(positions[0, i], velocities[0, i], accelerations[0, i])
    add_end_point = endpoint(positions[1, i], velocities[1, i], accelerations[1, i])
    add_points = np.vstack((add_start_point, add_end_point))
    point_arr = np.append(point_arr, add_points)
  point_arr = np.reshape(point_arr, (2, np.shape(positions)[1]))
  return point_arr

#call from another file
def perform_ja(traj, given_points=[], l=8.0):
  hJA = JA(traj, given_points, given_lambda=l)
  new_traj = hJA.generateTraj()
  del hJA
  return new_traj

def perform_ja_improved(traj, initial=[], end=[], lmbda=-1.0):
  #set up endpoints if none specified
  if not initial:
    initial = traj[0]
  if not end:
    end = traj[max(np.shape(traj)) - 1]
  #transpose if necessary
  if len(np.shape(traj)) > 1:
    if np.shape(traj)[0] > np.shape(traj)[1]:
      traj = np.transpose(traj)
  ntraj = np.reshape(traj, (1, max(np.shape(traj))))
  ## JA ##
  ja_fixed_points = generate_ja_fixed_points(np.array([[initial], [end]]))
  ja_traj = perform_ja(ntraj, ja_fixed_points, lmbda)
  ja_traj = np.reshape(ja_traj, np.shape(traj))
  #print(ja_traj[0])
  return ja_traj[0]

#in-file testing
def main():
  traj = np.array(np.linspace(1, 25, 100)).reshape(1, 100)
  hJA = JA(traj, 0, 5)
  new_traj = hJA.generateTraj()
  print(new_traj)

if __name__ == '__main__':
  main()