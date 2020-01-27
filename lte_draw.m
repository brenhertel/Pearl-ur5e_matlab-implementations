%%% perform lte on drawing %%%
filename = 'hello_trajectory.h5';

x = h5read(filename, '/hello/resampled_x');
y = h5read(filename, '/hello/resampled_y');

len = length(x);
traj = [x; y]';
lte_fixed_points = [1            ([-x(1)     -y(1)  ]);
                    len/2        ([x(len/2) y(len/2)]);
                    size(traj,1) ([x(len)   y(len)])];
[lte_deformed_x, lte_deformed_y] = lte(traj, lte_fixed_points);
plot(x, y, 'b.', lte_deformed_x, lte_deformed_y, 'r--');