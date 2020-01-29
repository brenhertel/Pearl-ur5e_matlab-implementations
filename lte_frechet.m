%%% automatically find segments in traj using frechet distance %%%
filename = 'hello_trajectory.h5';

x = h5read(filename, '/hello/resampled_x');
y = h5read(filename, '/hello/resampled_y');
window_increment = 0.01;
threshold = 0.1;
len = length(x);
traj = [x; y]';
%fix first point
lte_fixed_points = [1       ([x(1)   -1]);];
[lte_deformed_x, lte_deformed_y] = lte(traj, lte_fixed_points);
for iteration = 1:(1/window_increment - 1)
    for i = len*window_increment:(iteration+1)*len*window_increment
        d = calc_euclidean(x(i), y(i), lte_deformed_x(i), lte_deformed_y(i));
        if d > threshold
            disp('fixing points');
            lte_fixed_points = vertcat(lte_fixed_points, [i, ([x(i) y(i)])]);
            break;
        end
    end
    
    [lte_deformed_x, lte_deformed_y] = lte(traj, lte_fixed_points);
    disp(iteration)
    figure;
    plot(x, y, 'b.', lte_deformed_x, lte_deformed_y, 'r--');
    pause;
end


function max_d = calc_frechet(x1, y1, x2, y2, first, last)
distances = zeros(1, last-first);
for i=first:last
    distances(i) = calc_euclidean(x1(i), y1(i), x2(i), y2(i));
end
max_d = max(distances);
end

function d = calc_euclidean(x1, y1, x2, y2)
d = sqrt(power(x1 - x2, 2) + power(y1-y2, 2));
end