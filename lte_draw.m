%%% perform lte on drawing %%%
% filename = 'hello_trajectory.h5';
% 
% x = h5read(filename, '/hello/resampled_x');
% y = h5read(filename, '/hello/resampled_y');
filename = 'block_hello.h5';

x = h5read(filename, '/hello/x');
y = h5read(filename, '/hello/y');
% i = 1;
% figure;
% plot(x, y, 'b.');
% hold on;
% while i <= length(x)
%     disp(i);
%     plot(x(1:i), y(1:i), 'r.');
%     pause();
%     i = i+1; 
% end

%important_indeces = [156 313 349 413 482 505 569 635 717 786 853 883 924 974 1000];
%num_samples = 7;
%samples = linspace(1, length(important_indeces), num_samples);
%important_indeces = round(interp1(important_indeces, samples));
%disp(important_indeces);
len = length(x);
traj = [x; y]';

lte_fixed_points = [1                    ([x(1) y(1)]);
                    len                  ([x(len) y(len)])];
                    %important_indeces(1) ([x(important_indeces(1)) y(important_indeces(1))]);
                    %important_indeces(2) ([x(important_indeces(2)) y(important_indeces(2))]);
                    %important_indeces(3) ([x(important_indeces(3)) y(important_indeces(3))]);
                    %important_indeces(4) ([x(important_indeces(4)) y(important_indeces(4))]);
                    %important_indeces(5) ([x(important_indeces(5)) y(important_indeces(5))]);
                    %important_indeces(6) ([x(important_indeces(6)) y(important_indeces(6))]);
                    %important_indeces(7) ([x(important_indeces(7)) y(important_indeces(7))]);];
                    %important_indeces(8) ([x(important_indeces(8)) y(important_indeces(8))]);
                    %important_indeces(9) ([x(important_indeces(9)) y(important_indeces(9))]);
                    %important_indeces(10) ([x(important_indeces(10)) y(important_indeces(10))]);
                    %important_indeces(11) ([x(important_indeces(11)) y(important_indeces(11))]);
                    %important_indeces(12) ([x(important_indeces(12)) y(important_indeces(12))]);
                    %important_indeces(13) ([x(important_indeces(13)) y(important_indeces(13))]);
                    %important_indeces(14) ([x(important_indeces(14)) y(important_indeces(14))]);
                    %important_indeces(15) ([x(important_indeces(15)) y(important_indeces(15))]);];

[lte_deformed_x, lte_deformed_y] = lte(traj, lte_fixed_points);
plot(x, y, 'b.', lte_deformed_x, lte_deformed_y, 'r--');
figure;
for i = 1:len
    d = calc_euclidean(x(i), y(i), lte_deformed_x(i), lte_deformed_y(i));
    plot(i, d, 'g.');
    hold on;
end
function d = calc_euclidean(x1, y1, x2, y2) 
d = sqrt(power(x1 - x2, 2) + power(y1-y2, 2));
end