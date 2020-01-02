%%% Preprocessing position/rotation data from .h5 file %%%

%% Starting stuff %%
%filename = input('Please enter the complete filepath of the .h5 file: ', 's');
filename = 'C:\Users\Brendan\Downloads\recorded_demo Thu Dec 26 14_14_19 2019.h5';
data = h5read(filename, '/demo1/tf_info/pos_rot_data');

%pos_x_data = data(:,1);
%figure;
%for i = 1:length(pos_x_data)
%    plot(i, pos_x_data(i), 'k.');
%    hold on;
%end

%% Elminate  points at start and end when no movement happens %%
%look in sections of 10 points, if there is a significant change in the
%data within those 10 points, the demo has started. Do the same going from
%the back to find where the demo ends.
start_data = find_start(data);
end_data = find_end(data);
pos_x_data = data(start_data:end_data, 1);
pos_y_data = data(start_data:end_data, 2);
pos_z_data = data(start_data:end_data, 3);
rot_x_data = data(start_data:end_data, 4);
rot_y_data = data(start_data:end_data, 5);
rot_z_data = data(start_data:end_data, 6);
rot_w_data = data(start_data:end_data, 7);

%for i = 1:length(pos_x_data)
%    plot(i, pos_x_data(i), 'r.');
%    hold on;
%end

%% Start resampling process with spline fit %%
n = length(pos_x_data);
num_points = input('Please enter the number of points to be resampled from the trajectory: ');
s = linspace(1, n, num_points);
resampled_pos_x = spline(1:n, pos_x_data, s);
resampled_pos_y = spline(1:n, pos_y_data, s);
resampled_pos_z = spline(1:n, pos_z_data, s);
resampled_rot_x = spline(1:n, rot_x_data, s);
resampled_rot_y = spline(1:n, rot_y_data, s);
resampled_rot_z = spline(1:n, rot_z_data, s);
resampled_rot_w = spline(1:n, rot_w_data, s);

%for i = 1:length(resampled_pos_x)
%plot(s, resampled_pos_x, 'c-');
%hold on;
%end

%% Write the data to a .h5 file %%
index_end = max(strfind(filename, '\')) + 1;

h5create(['preprocessed_' filename(index_end:length(filename))], '/demo1/tf_info/pos_rot_data', [size(data, 2) length(resampled_pos_x)]) %might need to switch the sizes
h5write(['preprocessed_' filename(index_end:length(filename))], '/demo1/tf_info/pos_rot_data', [resampled_pos_x; resampled_pos_y; resampled_pos_z; resampled_rot_x; resampled_rot_y; resampled_rot_z; resampled_rot_w])
%We also need to write the joint states from the original file to the new
%one
js_data = h5read(filename, '/demo1/joint_state_info/joint_positions');
h5create(['preprocessed_' filename(index_end:length(filename))], '/demo1/joint_state_info/joint_positions', [size(js_data, 2) size(js_data, 1)])
h5write(['preprocessed_' filename(index_end:length(filename))], '/demo1/joint_state_info/joint_positions', js_data')

%% Functions to find the start and end of the dataset %%
function start = find_start(data)
n = length(data);
for i = 1:10:n
    for j = 1:size(data, 2)
        for k = 1:9
            dif = data(i, j) - data(i+k, j);
            if (abs(dif) > 0.001)
                %if it has moved more than 1 mm
                start = i;
                return;
            end
        end
    end
end
start = 1;
end

function end_demo = find_end(data)
n = length(data);
while (n > 10)
    for j = 1:size(data, 2)
        for k = 1:9
            dif = data(n, j) - data(n-k, j);
            if (abs(dif) > 0.001)
                %if it has moved more than 1 mm
                end_demo = n;
                return;
            end
        end
    end
    n = n - 10;
end
end_demo = n;
end