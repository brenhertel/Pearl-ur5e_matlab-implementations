function [lte_deformed_traj, ja_deformed_traj, dmp_deformed_traj] = perform_all_deformations(traj, time_data, initial, final, lambda)
%% set up fixed points %%
lte_fixed_points = [1            ([initial time_data(1)]);
                    size(traj,1) ([final   time_data(length(time_data))])];
ja_fixed_points = [initial; final];
disp(lte_fixed_points)
%% perform deformations %%
lte_traj = [traj; time_data]';
%disp(size(lte_traj));
[lte_deformed_traj, time_deformed_traj] = lte(lte_traj, lte_fixed_points);
dmp_deformed_traj = dmp(traj, time_data, 200, initial, final);
ja_deformed_traj  = filter_JA(traj',lambda,[],ja_fixed_points);

end