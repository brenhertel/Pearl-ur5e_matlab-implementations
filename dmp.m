function new_traj = dmp(traj, num_basis, initial, given_goal)
% %% Set up vel & acc %%
% for i = 1:(length(traj) - 1)
%     vel(i) = (traj(i + 1) - traj(i)) / (time_data(i + 1) - time_data(i));
% end
% vel(length(traj)) = 0;
% for i = 1:(length(vel) - 1)
%     acc(i) = (vel(i + 1) - vel(i)) / (time_data(i + 1) - time_data(i));
% end
% acc(length(traj)) = 0;
%vel = ones(size(traj));
%acc = vel;

%% Deformation %%

% general parameters
% dt        = 0.01;
% goal      = given_goal;
% tau       = 0.5;
% n_rfs     = num_basis;
% ID        = 1;
% 
% % initialize DMP
% dcp('clear',ID);
% dcp('init',ID,n_rfs,'my_dcp',1);
% 
% % use the in-built function to initialize the dcp with minjerk
% dcp('Batch_Fit',ID,tau,dt,traj',vel',acc');
% 
% % test the fit
% dcp('reset_state',ID, initial);
% dcp('set_goal',ID,goal,1);
% 
% for i=1:length(time_data)
%   
%   [y,~,~]=dcp('run',ID,tau,dt);
%   new_traj(i) = y;
% endhDMP_x = DiscreteDMP;
hDMP_y = DiscreteDMP;
hDMP_y.generate_DMP(num_basis);
%Use the given path to generate weights
hDMP_y.imitate_path(traj);
%change start and goal positions
hDMP_y.goal = given_goal;
hDMP_y.y_0 = initial;
%Generate deformed trajectory
new_traj = hDMP_y.rollout();

end
