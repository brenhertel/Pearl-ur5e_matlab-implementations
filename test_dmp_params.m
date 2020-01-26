%%% Test dcp parameters %%%

%% Starting stuff %%
filename = 'lasa_dataset.h5';
pos_data = h5read(filename, '/Angle/demo1/pos');
pos_x_data = pos_data(1, :);
pos_y_data = pos_data(2, :);
time_data = h5read(filename, '/Angle/demo1/t');

%% Set up vel & acc %%
for i = 1:(length(pos_x_data) - 1)
    vel_x(i) = (pos_x_data(i + 1) - pos_x_data(i)) / (time_data(i + 1) - time_data(i));
end
vel_x(length(pos_x_data)) = 0;
for i = 1:(length(vel_x) - 1)
    acc_x(i) = (vel_x(i + 1) - vel_x(i)) / (time_data(i + 1) - time_data(i));
end
acc_x(length(pos_x_data)) = 0;

for i = 1:(length(pos_y_data) - 1)
    vel_y(i) = (pos_y_data(i + 1) - pos_y_data(i)) / (time_data(i + 1) - time_data(i));
end
vel_y(length(pos_y_data)) = 0;
for i = 1:(length(vel_y) - 1)
    acc_y(i) = (vel_y(i + 1) - vel_y(i)) / (time_data(i + 1) - time_data(i));
end
acc_y(length(pos_y_data)) = 0;

%% Deformation %%

dt = 0.1;
tau = 0.5;
n_rfs = 100;
optimal_dt = dt;
optimal_tau = tau;
optimal_n_rfs = n_rfs;

% alpha_z = 1;
% beta_z  = 1;
% alpha_g = 1;
% alpha_x = 1;
% alpha_v = 1;
% beta_v  = 1;
%
% optimal_alpha_z = alpha_z;
% optimal_beta_z  = beta_z;
% optimal_alpha_g = alpha_g;
% optimal_alpha_x = alpha_x;
% optimal_alpha_v = alpha_v;
% optimal_beta_v  = beta_v;

%Run 1 Results:
optimal_alpha_z = 105;
%optimal_beta_z  = 6.3;
optimal_beta_z = alpha_z / 4;
optimal_alpha_g = .01;
optimal_alpha_x = .01;
optimal_alpha_v = .01;
optimal_beta_v  = .01;

alpha_z = optimal_alpha_z;
beta_z  = optimal_beta_z;
alpha_g = optimal_alpha_g;
alpha_x = optimal_alpha_x;
alpha_v = optimal_alpha_v;
beta_v  = optimal_beta_v;

base_x = dmp_xtra(pos_x_data, vel_x, acc_x, n_rfs, alpha_z, beta_z, alpha_g, alpha_x, alpha_v, beta_v, dt, tau);
base_y = dmp_xtra(pos_y_data, vel_y, acc_y, n_rfs, alpha_z, beta_z, alpha_g, alpha_x, alpha_v, beta_v, dt, tau);

base_dist_diff = calc_sum_dist(pos_x_data, pos_y_data, base_x, base_y);
optimal_dist_diff = base_dist_diff;
fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
    alpha_z, beta_z, alpha_g, alpha_x, alpha_v, beta_v, base_dist_diff);

best_curr_dist = base_dist_diff;
for n_rfs = 100:500
    x = dmp_xtra(pos_x_data, vel_x, acc_x, n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_dt, optimal_tau);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, n_rfs, alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_dt, optimal_tau);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, n_rfs = %d, dt = %f, tau = %f\n path difference = %f\n', ...
        alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, n_rfs, optimal_dt, optimal_tau, dist_diff);
    if dist_diff < best_curr_dist
        optimal_n_rfs = n_rfs;
        best_curr_dist = dist_diff;
    end
end
best_curr_dist = base_dist_diff;
for dt = 0.1:0.1:10
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, dt, optimal_tau);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, dt, optimal_tau);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, n_rfs = %d, dt = %f, tau = %f\n path difference = %f\n', ...
        alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_n_rfs, dt, optimal_tau, dist_diff);
    if dist_diff < best_curr_dist
        optimal_dt = dt;
        best_curr_dist = dist_diff;
    end
end
best_curr_dist = base_dist_diff;
for tau = 0.1:0.1:10
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_dt, tau);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_dt, tau);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, n_rfs = %d, dt = %f, tau = %f\n path difference = %f\n', ...
        alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, optimal_n_rfs, optimal_dt, tau, dist_diff);
    if dist_diff < best_curr_dist
        optimal_tau = tau;
        best_curr_dist = dist_diff;
    end
end

best_curr_dist = base_dist_diff;
for alpha_z = 100:0.1:110
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, dist_diff);
    if dist_diff < best_curr_dist
        optimal_alpha_z = alpha_z;
        best_curr_dist = dist_diff;
    end
end

best_curr_dist = base_dist_diff;
for beta_z = 1.3:0.1:11.3
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        optimal_alpha_z, beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, dist_diff);
    best_curr_dist = base_dist_diff;
    if dist_diff < best_curr_dist
        optimal_beta_z = beta_z;
    end
end

best_curr_dist = base_dist_diff;
for alpha_g = 0.01:0.01:2
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        optimal_alpha_z, optimal_beta_z, alpha_g, optimal_alpha_x, optimal_alpha_v, optimal_beta_v, dist_diff);
    if dist_diff < best_curr_dist
        optimal_alpha_g = alpha_g;
        best_curr_dist = dist_diff;
    end
end

best_curr_dist = base_dist_diff;
for alpha_x = 0.01:0.01:2
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        alpha_x, optimal_alpha_v, optimal_beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        alpha_x, optimal_alpha_v, optimal_beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        optimal_alpha_z, optimal_beta_z, optimal_alpha_g, alpha_x, optimal_alpha_v, optimal_beta_v, dist_diff);
    if dist_diff < best_curr_dist
        optimal_alpha_x = alpha_x;
        best_curr_dist = dist_diff;
    end
end

best_curr_dist = base_dist_diff;
for alpha_v = 0.01:0.01:2
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, alpha_v, optimal_beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, alpha_v, optimal_beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        optimal_alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, alpha_v, optimal_beta_v, dist_diff);
    if dist_diff < best_curr_dist
        optimal_alpha_v = alpha_v;
        best_curr_dist = dist_diff;
    end
end

best_curr_dist = base_dist_diff;
for beta_v = 0.01:0.01:2
    x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, beta_v);
    y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
        optimal_alpha_x, optimal_alpha_v, beta_v);
    
    dist_diff = calc_sum_dist(pos_x_data, pos_y_data, x, y);
    fprintf('alpha_z = %f, beta_z  = %f, alpha_g = %f,\n alpha_x = %f, alpha_v = %f, beta_v  = %f,\n, path difference = %f\n', ...
        optimal_alpha_z, optimal_beta_z, optimal_alpha_g, optimal_alpha_x, optimal_alpha_v, beta_v, dist_diff);
    if dist_diff < best_curr_dist
        optimal_beta_v = beta_v;
        best_curr_dist = dist_diff;
    end
end

x = dmp_xtra(pos_x_data, vel_x, acc_x, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
    optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
y = dmp_xtra(pos_y_data, vel_y, acc_y, optimal_n_rfs, optimal_alpha_z, optimal_beta_z, optimal_alpha_g, ...
    optimal_alpha_x, optimal_alpha_v, optimal_beta_v);
plot(pos_x_data, pos_y_data, 'b', x, y, 'c');
function dist_diff = calc_sum_dist(x1, y1, x2, y2)
dist_diff = 0;
for i = 1:length(x1)
    dist_diff = dist_diff + sqrt(power(x1(i) - x2(i), 2) + power(y2(i) - y1(i), 2));
end
end

function new_traj = dmp_xtra(traj, vel, acc, num_basis, alpha_z, beta_z, alpha_g, alpha_x, alpha_v, beta_v,dt, tau)
%% Deformation %%

% general parameters
%dt        = 0.01;
%tau       = 0.5;
n_rfs     = num_basis;
ID        = 1;
goal      = traj(length(traj));
initial   = traj(1);

% initialize DMP
dcp('clear',ID);
dcp('init',ID,n_rfs,'my_dcp',1, alpha_z, beta_z, alpha_g, alpha_x, alpha_v, beta_v);

% use the in-built function to initialize the dcp with minjerk
dcp('Batch_Fit',ID,tau,dt,traj',vel',acc');

% test the fit
dcp('reset_state',ID, initial);
dcp('set_goal',ID,goal,1);

for i=1:length(traj)
    
    [y,~,~]=dcp('run',ID,tau,dt);
    new_traj(i) = y;
end

end