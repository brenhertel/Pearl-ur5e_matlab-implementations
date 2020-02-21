%%% store drawing in .h5 file %%%
%% preprocess trajs %%
x_traj = xx;
y_traj = yy;
num_points = 1000;
x_traj = preprocess_traj(x_traj, num_points);
y_traj = preprocess_traj(y_traj, num_points);
%% store in file %%
name = 'Circle';
h5create([name '_drawing_demo.h5'],['/' name '/x'],[1 num_points])
h5write([name '_drawing_demo.h5'],['/' name '/x'], x_traj)
h5create([name '_drawing_demo.h5'],['/' name '/y'],[1 num_points])
h5write([name '_drawing_demo.h5'],['/' name '/y'], y_traj)