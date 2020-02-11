function new_traj = preprocess_traj(traj, num_points)
n = length(traj);
s = linspace(1, n, num_points);
new_traj = spline(1:n, traj, s);
end