%%% matlab draw 2 %%%
figure;
xlim([0 1]);
ylim([0 1]);
hold on;
x_traj = [];
y_traj = [];
while(1)
    [x, y] = ginput(1);
    plot(x, y, 'bo');
    x_traj = [x_traj; x];
    y_traj = [y_traj; y];
    f = fit(x_traj, y_traj, 'poly100');
    plot(f, 'r--')
end