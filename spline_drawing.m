%%% spline draw %%%
filename = 'hello2.h5';

x = h5read(filename, '/hello/x');
y = h5read(filename, '/hello/y');

plot(x, y)
hold on;

n = length(x);
%num_points = input('Please enter the number of points to be resampled from the trajectory: ');
num_points = 1000;
s = linspace(1, n, num_points);
resampled_x = spline(1:n, x, s);
resampled_y = spline(1:n, y, s);

plot(resampled_x, resampled_y, 'r--');


h5create(filename,'/hello/resampled_x',[1 num_points])
h5write(filename,'/hello/resampled_x',resampled_x)
h5create(filename,'/hello/resampled_y',[1 num_points])
h5write(filename,'/hello/resampled_y',resampled_y)