%%% store drawing in .h5 file %%%
%% preprocess trajs %%
name = 'Straight_Ribbon';
x_traj = x_new * 100;
y_traj = y_new * 100;
num_points = 1000;
x_traj = preprocess_traj(x_traj, num_points);
y_traj = preprocess_traj(y_traj, num_points);

%% plot %%
plot(x_traj, y_traj)
title(name)
pause

%% store in file %%
filename = [name '_drawing_demo.h5'];
h5create(filename,['/' name '/x'],[1 num_points])
h5write(filename,['/' name '/x'], x_traj)
h5create(filename,['/' name '/y'],[1 num_points])
h5write(filename,['/' name '/y'], y_traj)

% move the file to proper folder
movefile(filename, 'h5 files/');