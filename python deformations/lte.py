#!/usr/bin/env python

#python (to be used in ros) implementation of laplacian trajectory editing.
#Original: www.itr.ei.tum.de/fileadmin/w00bok/www/CodeExamples/laplacianHardConstraints.m
#Each LTE object is used to create a deformed trajectory for any 1D given trajectory

import numpy as np

#for some reason np.diag wasn't working properly (instead of creating a diagonal of the array it was just returning the same array) so I created my own function to diagonalize matrices
def my_diag(A, offset=0):
  for i in range (np.size(A, 0)):
    if i + offset < np.size(A, 1) and i + offset >= 0:
      A[i][i + offset] = 1
  return A

#I use the point objects to define fixed points along the deformed trajectory
class point(object):
  def __init__(self, given_index, given_x):
    self.index = given_index;
    self.x = given_x;

#Use an object to organize the LTE process
class LTE(object):
  def __init__(self, given_traj, given_points=0, given_weight=1e9, given_boundary_conds=0):
    #given trajectory is dim x Nodes, want to make it nodes * dim
    self.traj = np.transpose(given_traj)
    self.trajMod = self.traj
    self.fixedPos = given_points
    #if no fixed points are given, use the initial and final points of the given trajectory
    if given_points == 0:
      pos = []
      pos.append(point(0, self.traj[0]))
      pos.append(point(len(self.traj) - 1, self.traj[len(self.traj) - 1]))
      self.fixedPos = pos
    self.fixedWeight = given_weight
    self.boundCond = given_boundary_conds

  def generateDelta(self):
    delta = np.zeros((np.size(self.traj, 0), 1))
    for i in range (len(delta)):
      matrix_sum = 0
      for j in range (len(self.L)):
        matrix_sum = matrix_sum + (self.L[i][j] * self.traj[j])
      delta[i] = matrix_sum
    self.delta = delta

  #Create the laplacian, delta, and constraint matrices needed for calculation of trajectory
  def generateLaplacian(self):
    nbNodes = np.size(self.traj, 0)
    L = np.zeros((nbNodes, nbNodes))
    L = my_diag(L)
    L = L * 2
    A = np.zeros((nbNodes, nbNodes))
    A = my_diag(A, -1)
    L = L - A
    B = np.zeros((nbNodes, nbNodes))
    B = my_diag(B, 1)
    L = L - B    
    L[0][1] = -2.0
    L[nbNodes - 1][nbNodes - 2] = -2.0
    L = L / 2
    self.L = L
    self.generateDelta()
    if self.boundCond == 0:
      self.L = np.delete(self.L, 0, 0)
      self.L = np.delete(self.L, np.size(self.L, 0) - 1, 0)
      self.delta = np.delete(self.delta, 0, 0)
      self.delta = np.delete(self.delta, np.size(self.delta, 0) - 1, 0)
    for i in range (np.size(self.fixedPos)):
      to_append_L = np.zeros(nbNodes)
      to_append_L[self.fixedPos[i].index] = self.fixedWeight
      self.L = np.vstack((self.L, to_append_L))
      to_append_delta = np.zeros(1)
      to_append_delta = self.fixedWeight * self.fixedPos[i].x
      self.delta = np.vstack((self.delta, to_append_delta))
  
  #calculate trajectory
  def generateTraj(self):
    new_traj = np.linalg.solve(self.L, self.delta)
    return new_traj

#this function does all the outside steps, and was made to be called from another file
#Given a trajectory, and if necessary, fixed points, it will return the lte deformed trajectory 
def perform_lte(traj, given_points=0):
  hLTE = LTE(traj, given_points)
  hLTE.generateLaplacian()
  new_traj = hLTE.generateTraj()
  return new_traj

#Function to create fixed points for lte deformation. Given an array of indeces and positions it will use the point object which the lte object uses to create an array of points to return
def generate_lte_fixed_points(indeces, positions):
  point_arr = []
  for i in range (len(indeces)):
    add_point = point(indeces[i], positions[i])
    point_arr.append(add_point)
  return point_arr

def perform_lte_improved(traj, initial=[], end=[]):
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
  ## LTE ##
  indeces = [1, max(np.shape(traj)) - 1]
  lte_fixed_points = generate_lte_fixed_points(indeces, [initial, end])
  lte_traj = perform_lte(ntraj, lte_fixed_points)
  lte_traj = np.reshape(lte_traj, np.shape(traj))
  #print(lte_traj)
  return lte_traj[0]

#in-file testing
def main():
  hLTE = LTE(np.ones((1, 5)))
  hLTE.generateLaplacian()
  new_traj = hLTE.generateTraj()
  #print(new_traj)

if __name__ == '__main__':
  main()