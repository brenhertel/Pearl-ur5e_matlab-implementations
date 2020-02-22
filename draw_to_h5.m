%%% store drawing in .h5 file %%%
%% preprocess trajs %%
x_traj = xx * 100;
y_traj = yy * 100;
num_points = 1000;
x_traj = preprocess_traj(x_traj, num_points);
y_traj = preprocess_traj(y_traj, num_points);
%% store in file %%
name = 'Circle';
filename = [name '_drawing_demo.h5'];
h5create(filename,['/' name '/x'],[1 num_points])
h5write(filename,['/' name '/x'], x_traj)
h5create(filename,['/' name '/y'],[1 num_points])
h5write(filename,['/' name '/y'], y_traj)

% move the file to proper folder
movefile(filename, 'h5 files/');