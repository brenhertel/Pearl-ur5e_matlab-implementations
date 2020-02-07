#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import lte
import ja
import dmp

lasa_names = ['Angle','BendedLine','CShape','DoubleBendedLine','GShape', \
              'heee','JShape','JShape_2','Khamesh','Leaf_1', \
              'Leaf_2','Line','LShape','NShape','PShape', \
              'RShape','Saeghe','Sharpc','Sine','Snake', \
              'Spoon','Sshape','Trapezoid','Worm','WShape', \
              'Zshape','Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4']

def get_lasa_traj1(shape_name):
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    shape = hf.get(shape_name)
    demo = shape.get('demo1')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    #close out file
    hf.close()
    return [x_data, y_data]

def get_lasa_traj2_initial(shape_name):
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    #open the file
    filename = '../h5 files/lasa_dataset.h5'
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    shape = hf.get(shape_name)
    demo = shape.get('demo2')
    pos_info = demo.get('pos')
    pos_data = np.array(pos_info)
    y_data = np.delete(pos_data, 0, 1)
    x_data = np.delete(pos_data, 1, 1)
    x0 = x_data[0]
    y0 = y_data[0]
    #close out file
    hf.close()
    return [x0, y0]

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
  dmp_traj = dmp.perform_dmp(traj, [initial, end])
  return [lte_traj, ja_traj, dmp_traj]


def main():
  global lasa_names
  plt.figure(0)
  for i in range (5):#(len(lasa_names)):
    print(lasa_names[i])
    print(i)
    print(i / 5)
    print(i % 5)
    y_axis = (i % 5)
    x_axis = (i // 5)
    [x_data, y_data] = get_lasa_traj1(lasa_names[i])
    ax = plt.subplot2grid((6, 5), (x_axis, y_axis))
    ax.plot(x_data, y_data)
    [x0, y0] = get_lasa_traj2_initial(lasa_names[i])
    #ax.title(lasa_names[i])
    [x_lte, x_ja, x_dmp] = perform_all_deformations(x_data, x0, lmbda=15.0)
    [y_lte, y_ja, y_dmp] = perform_all_deformations(y_data, y0, lmbda=15.0)
    ax.plot(x_lte, y_lte)
    ax.plot(x_ja, y_ja)
    ax.plot(x_dmp, y_dmp)
  plt.show()


if __name__ == '__main__':
  main()