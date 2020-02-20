%%% ja optimization for trajectory %%%
function opt_lambda = optimize_ja(traj1)
global traj;
traj = traj1;
%filename = 'lasa_dataset.h5';
%pos_data = h5read(filename, '/Angle/demo1/pos');
%pos_x_data = pos_data(1, :);
%pos_y_data = pos_data(2, :);
%traj = [pos_x_data; pos_y_data]';
fun = @get_ja_fd;
% opt_lambda = fminunc(fun, (numel(traj)/20));
options = optimset('Display', 'final', 'TolX', 1e-8);
opt_lambda = fminbnd(fun, 0, 250, options);
[ja_deformed_x, ja_deformed_y] = filter_JA(traj, opt_lambda);
figure;
plot(traj(:, 1), traj(:, 2), 'k', ja_deformed_x(:, 1), ja_deformed_y(:, 1), 'r--');
%disp(opt_lambda)
end

function fd = get_ja_fd(lambda)
global traj;
%disp('lambda: ');
%disp(lambda);
[ja_deformed_x, ja_deformed_y] = filter_JA(traj, lambda);
traj2 = [ja_deformed_x(:, 1) ja_deformed_y(:, 1)];
[fd, ~] = DiscreteFrechetDist(traj, traj2);
%disp('fd: ');
%disp(fd);
end