#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
#import perform_pydmps

import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'C:/Users/BH/Documents/GitHub/Pearl-ur5e_matlab-implementations/python deformations/dmp_for_comparison/')

import perform_new_dmp as pnd

def get_lasa_traj():
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    shape = hf.get('Angle')
    demo = shape.get('demo1')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    #close out file
    hf.close()
    return [x_data, y_data]

def perform_all_deformations(traj, initial=[], end=[], lmbda=-1.0):
  #set up endpoints if none specified
  if not initial:
    initial = traj[0]
  if not end:
    end = traj[max(np.shape(traj)) - 1]
  #transpose if necessary
  if len(np.shape(traj)) > 1:
    if np.shape(traj)[0] > np.shape(traj)[1]:
      traj = np.transpose(traj)
  traj = np.reshape(traj, (1, max(np.shape(traj))))
  ## LTE ##
  indeces = [1, max(np.shape(traj)) - 1]
  lte_fixed_points = lte.generate_lte_fixed_points(indeces, [initial, end])
  lte_traj = lte.perform_lte(traj, lte_fixed_points)
  ## JA ##
  ja_fixed_points = ja.generate_ja_fixed_points(np.array([[initial], [end]]))
  ja_traj = ja.perform_ja(traj, ja_fixed_points, lmbda)
  ## DMP ##
  traj = np.reshape(traj, (np.size(traj)))
  new_dt = 1.0 / np.size(traj)
  dmp_traj = pnd.perform_new_dmp_adapted(traj, initial, end, dt=new_dt)
  return [lte_traj, ja_traj, dmp_traj]

def perform_all_deformations_no_dmp(traj, initial=[], end=[], lmbda=-1.0):
  #set up endpoints if none specified
  if not initial:
    initial = traj[0]
  if not end:
    end = traj[max(np.shape(traj)) - 1]
  #transpose if necessary
  if len(np.shape(traj)) > 1:
    if np.shape(traj)[0] > np.shape(traj)[1]:
      traj = np.transpose(traj)
  traj = np.reshape(traj, (1, max(np.shape(traj))))
  ## LTE ##
  indeces = [1, max(np.shape(traj)) - 1]
  lte_fixed_points = lte.generate_lte_fixed_points(indeces, [initial, end])
  lte_traj = lte.perform_lte(traj, lte_fixed_points)
  ## JA ##
  ja_fixed_points = ja.generate_ja_fixed_points(np.array([[initial], [end]]))
  ja_traj = ja.perform_ja(traj, ja_fixed_points, lmbda)
  return [lte_traj, ja_traj]

def main():
  ## Undeformed ##
  #retrive data from file
  [x_data, y_data] = get_lasa_traj()

  ## LTE ##
  #set up fixed points for lte deformations
  indeces = [1, len(x_data) - 1]
  x_positions = [-50, -5]
  y_positions = [5, 5]
  x_fixed_points = lte.generate_lte_fixed_points(indeces, x_positions)
  y_fixed_points = lte.generate_lte_fixed_points(indeces, y_positions)
  #perform lte deformations
  x_lte_traj = lte.perform_lte(np.transpose(x_data), x_fixed_points)
  y_lte_traj = lte.perform_lte(np.transpose(y_data), y_fixed_points)
  
  ## JA ##
  #fixed points
  x_fixed_points = ja.generate_ja_fixed_points(np.array([[-50], [-5]]))
  y_fixed_points = ja.generate_ja_fixed_points(np.array([[5], [5]]))
  #perform the deformations
  lmbda = 10.0 #Good lambda values are from ~6.0 to ~15.0
  x_ja_traj = ja.perform_ja(np.transpose(x_data), x_fixed_points, lmbda)
  y_ja_traj = ja.perform_ja(np.transpose(y_data), y_fixed_points, lmbda)

  ## DMP ##
  x_dmp_traj = dmp.perform_dmp(np.transpose(x_data), [-50, -5])
  #y_dmp_traj = dmp.perform_dmp(np.transpose(y_data), [5, 5])
  y_dmp_traj = dmp.perform_dmp(np.transpose(y_data))

  #plot data
  plt.plot(x_data, y_data)
  plt.plot(x_lte_traj, y_lte_traj)
  plt.plot(x_ja_traj, y_ja_traj)
  plt.plot(x_dmp_traj, y_dmp_traj)
  plt.show()
  #testing perform_all function
  [x_lte, x_ja, x_dmp] = perform_all_deformations(x_data, -50, -5)
  [y_lte, y_ja, y_dmp] = perform_all_deformations(y_data, -5, 5)
  plt.plot(x_data, y_data)
  plt.plot(x_lte, y_lte)
  plt.plot(x_ja, y_ja)
  plt.plot(x_dmp, y_dmp)
  plt.show()
  return

if __name__ == '__main__':
  main()