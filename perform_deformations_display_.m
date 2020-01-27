%%% Graph the dmp, lte, and ja transformations for a dataset from a .h5
%%% file

%% Starting stuff %%
filename = 'lasa_dataset.h5';
pos_data = h5read(filename, '/Angle/demo1/pos');
pos_x_data = pos_data(1, :);
pos_y_data = pos_data(2, :);
t_data = h5read(filename, '/Angle/demo1/t');

%% dmp deformation %%
% %create DMP objects
% hDMP_x = DiscreteDMP;
% hDMP_y = DiscreteDMP;
% %generate DMP with 200 basis functions (increasing this number increases
% %how closely the generated trajectory follows the given trajectory)
% hDMP_x.generate_DMP(100);
% %Use the given path to generate weights
% dmp_imitate_x = hDMP_x.imitate_path(pos_x_data);
% %change start and goal positions
% hDMP_x.goal = -5;
% hDMP_x.y_0 = -50;
% %Generate deformed trajectory
% dmp_deformed_x = hDMP_x.rollout();
% %again for y
% hDMP_y.generate_DMP(1000);
% dmp_imitate_y = hDMP_y.imitate_path(pos_y_data);
% hDMP_y.goal = 5;
% hDMP_y.y_0 = 5;
% dmp_deformed_y = hDMP_y.rollout();
%dmp_deformed_x = dmp(pos_x_data, t_data, 100, -50, -5);
%dmp_deformed_y = dmp(pos_y_data, t_data, 100, 5, 5);

D = 1;
K = 1;
dt = 0.001;
cs_alpha = 1;
tau = 1;

hDMP_x = pastor_dmp;
hDMP_y = pastor_dmp;
%generate DMP with 200 basis functions (increasing this number increases
%how closely the generated trajectory follows the given trajectory)
hDMP_x.generate_DMP(2, D, K, cs_alpha, dt, tau);
%Use the given path to generate weights
dmp_imitate_x = hDMP_x.imitate_path(pos_x_data);
%change start and goal positions
hDMP_x.goal = -5;
hDMP_x.y_0 = -50;
%Generate deformed trajectory
dmp_deformed_x = hDMP_x.rollout();
%again for y
hDMP_y.generate_DMP(2, D, K, cs_alpha, dt, tau);
dmp_imitate_y = hDMP_y.imitate_path(pos_y_data);
hDMP_y.goal = 5;
hDMP_y.y_0 = 5;
dmp_deformed_y = hDMP_y.rollout();

%% lte deformation %%
traj = [pos_x_data; pos_y_data]';
disp(size(traj));
lte_fixed_points = [1            ([-50 5]);
                    size(traj,1) ([-5  5])];
[lte_deformed_x, lte_deformed_y] = lte(traj, lte_fixed_points);

%% ja deformation %%
lambda = 5;
ja_fixed_points = [-50 5; -5 5];
[ja_deformed_x, ja_deformed_y] = filter_JA(traj, lambda, [], ja_fixed_points);

%% display all 4 (original, dmp, lte, ja) %%
fh = figure;
plot(pos_x_data, pos_y_data, 'k');
hold on;
plot(dmp_deformed_x, dmp_deformed_y, 'r--');
plot(lte_deformed_x, lte_deformed_y, 'g');
plot(ja_deformed_x(:, 1), ja_deformed_y(:, 1), 'b');