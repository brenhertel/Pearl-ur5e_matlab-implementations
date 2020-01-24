%%% test dcps %%%
clear all;
%% Starting stuff %%
filename = 'lasa_dataset.h5';
pos_data = h5read(filename, '/Angle/demo1/pos');
pos_x_data = pos_data(1, :);
pos_y_data = pos_data(2, :);
t_data = h5read(filename, '/Angle/demo1/t');

%% Set up vel & acc %%
for i = 1:(length(pos_x_data) - 1)
    vel(i) = (pos_x_data(i + 1) - pos_x_data(i)) / (t_data(i + 1) - t_data(i));
end
vel(length(pos_x_data)) = 0;
for i = 1:(length(vel) - 1)
    acc(i) = (vel(i + 1) - vel(i)) / (t_data(i + 1) - t_data(i));
end
acc(length(pos_x_data)) = 0;

%% Deformation %%

% general parameters
dt        = 0.001;
goal      = pos_x_data(length(pos_x_data));
%goal = -10;
tau       = 0.5;
n_rfs     = 10;
ID        = 1;

% initialize DMP
dcp('clear',ID);
dcp('init',ID,n_rfs,'minJerk_dcp',1);

% use the in-built function to initialize the dcp with minjerk
dcp('Batch_Fit',ID,tau,dt,pos_x_data',vel',acc');

% test the fit
dcp('reset_state',ID, pos_x_data(1));
dcp('set_goal',ID,goal,1);

for i=1:length(t_data)
  
  [y,yd,ydd]=dcp('run',ID,tau,dt);
  traj(i) = y;
end

%% Plot Results %%
figure;
plot(t_data, pos_x_data);
hold on;
plot(t_data, traj, 'r--');
