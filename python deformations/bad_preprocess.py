#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import h5py

def preprocess_1d(traj, num_points, start=-1, end=-1):
  print(np.shape(traj))
  traj = np.reshape(traj, (1, max(np.shape(traj))))
  #print(traj)
  #I want to shave off the start and end where no motion is happening
  if start < 0:
    start = find_start(traj, max(np.shape(traj)))
  if end < 0:
    end = find_end(traj, max(np.shape(traj)))
  data = []
  for i in range (max(np.shape(traj))):
    if i >= start and i <= end:
      data.append(traj[0, i])
  return [data, start, end]

def find_start(traj, n):
  traj = np.reshape(traj, (1, n))
  #print(np.shape(traj))
  i = 0
  while i < n:
    k = 1
    while k < 10 and i + k < n:
      dif = traj[0, i] - traj[0, i + k]
      if abs(dif) > 0.001:
        return i
      k = k + 1
    i = i + 10
  return 0

def find_end(traj, n):
  traj = np.reshape(traj, (1, n))
  print(np.shape(traj))
  i = n - 1
  while i > 10:
    k = 1
    while k < 10:
      dif = traj[0, i] - traj[0, i-k]
      if abs(dif) > 0.001:
        return i
      k = k + 1
    i = i - 10
  return n

def preprocess_nd(data, num_points, start=-1, end=-1):
  dims = min(np.shape(data))
  #get starts
  if start < 0:
    starts = np.zeros(dims)
    for i in range (dims):
      starts[i] = find_start(data[i], max(np.shape(data[i])))
    actual_start = min(starts)
  else:
    actual_start = start
  #print('actual start: %d' % actual_start)
  #get end
  if end <= 0:
    ends = np.zeros(dims)
    for i in range (dims):
      ends[i] = find_end(data[i], max(np.shape(data[i])))
    actual_end = max(ends)
  else:
    actual_end = end
  num_points = 1 + actual_end - actual_start
  #print('actual end: %d' % actual_end)
  #preprocess
  resample_data = []
  for i in range (dims):
    #print('resampling row %d' % i)
    [data_new, s, e] = preprocess_1d(data[i], num_points, actual_start, actual_end)
    resample_data.append(data_new)
  print(np.shape(resample_data))
  print((dims, num_points))
  resample_data = np.reshape(resample_data, (int(dims), int(num_points)))
  return [resample_data, actual_start, actual_end]

def main():
  #get filename from user
  #filename = raw_input('Enter the filename of the .h5 demo: ')
  filename = '3d_demo.h5'
  #open the file
  hf = h5py.File(filename, 'r')
  #navigate to necessary data and store in numpy arrays
  #first layer
  demo = hf.get('demo1')
  #second layer
  time_info = demo.get('time_info')
  js_info = demo.get('joint_state_info')
  tf_info = demo.get('tf_info')
  gripper_info = demo.get('gripper_info')
  #third layer
  time_data = time_info.get('time_data')
  jp_data = js_info.get('joint_positions')
  je_data = js_info.get('joint_effort')
  pos_rot_data = tf_info.get('pos_rot_data')
  force_data = gripper_info.get('force_data')
  torq_data = gripper_info.get('torque_data')
  #convert to numpy arrays
  time_data = np.array(time_data)
  jp_data = np.array(jp_data)
  je_data = np.array(je_data)
  pos_rot_data = np.array(pos_rot_data)
  force_data = np.array(force_data)
  torq_data = np.array(torq_data)
  #close out file
  hf.close()
  #convert ns to secs and add
  time = time_data[0]
  for i in range (len(time)):
    time[i] = time[i] + (time_data[1, i] * 0.000000001)
  time = np.reshape(time, (1, len(time)))
  print(time)
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))
  num_points = 100
  [pos_rot_data, actual_start, actual_end] = preprocess_nd(pos_rot_data, num_points)#, start=-1, end=-1)
  print(np.shape(pos_rot_data))
  print(actual_start)
  print(actual_end)
  time = preprocess_nd(time, num_points, actual_start, actual_end)[0]
  jp_data = preprocess_nd(jp_data, num_points, actual_start, actual_end)[0]
  je_data = preprocess_nd(je_data, num_points, actual_start, actual_end)[0]
  force_data = preprocess_nd(force_data, num_points, actual_start, actual_end)[0]
  torq_data = preprocess_nd(torq_data, num_points, actual_start, actual_end)[0]
  print(np.shape(time))
  print(np.shape(jp_data))
  print(np.shape(je_data))
  print(np.shape(pos_rot_data))
  print(np.shape(force_data))
  print(np.shape(torq_data))

  fp = h5py.File ('bad preprocessed ' + filename, 'w')
  demo_name = 'demo1'
  dset_time = fp.create_dataset(demo_name + '/time_info/time_data', data=time)
  dset_joint = fp.create_dataset(demo_name + '/joint_state_info/joint_positions', data=jp_data)
  dset_eff = fp.create_dataset(demo_name + '/joint_state_info/joint_effort', data=je_data)
  dset_pos = fp.create_dataset(demo_name + '/tf_info/pos_rot_data', data=pos_rot_data)
  dset_force = fp.create_dataset(demo_name + '/gripper_info/force_data', data=force_data)
  dset_torq = fp.create_dataset(demo_name + '/gripper_info/torque_data', data=torq_data)
  fp.close()


if __name__ == '__main__':
  main()