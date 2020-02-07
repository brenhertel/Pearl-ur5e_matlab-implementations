%%% deform lasa %%%

names = {'Angle','BendedLine','CShape','DoubleBendedLine','GShape',...
    'heee','JShape','JShape_2','Khamesh','Leaf_1',...
    'Leaf_2','Line','LShape','NShape','PShape',...
    'RShape','Saeghe','Sharpc','Sine','Snake',...
    'Spoon','Sshape','Trapezoid','Worm','WShape','Zshape',...
    'Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4'};

filename = 'h5 files/lasa_dataset.h5';

lambda = [];

for i = 1:5%length(names)
    pos_data = h5read(filename, ['/' names{i} '/demo1/pos']);
    pos_x_data{i} = pos_data(1, :);
    pos_y_data{i} = pos_data(2, :);
    t_data = h5read(filename, ['/' names{i} '/demo1/t']);
    pos2_data = h5read(filename, ['/' names{i} '/demo2/pos']);
    initial_x = pos2_data(1, 1);
    initial_y = pos2_data(2, 1);
    final_x = pos_x_data{i}(length(pos_x_data{i}));
    final_y = pos_y_data{i}(length(pos_y_data{i}));
    
    %% dmp deformation %%
    dmp_deformed_x{i} = dmp(pos_x_data{i}, 200, initial_x, final_x);
    dmp_deformed_y{i} = dmp(pos_y_data{i}, 200, initial_y, final_y);
    
    %% lte deformation %%
    %traj = [pos_x_data{i}; pos_y_data{i}]';
    %disp(size(traj));
    %lte_fixed_points = [1            ([initial_x initial_y]);
    %    size(traj,1) ([final_x   final_y])];
    %[lte_deformed_x{i}, lte_deformed_y{i}] = lte(traj, lte_fixed_points);
    
    %% ja deformation %%
    %lambda = 5;
    %ja_fixed_points = [initial_x initial_y; final_x final_y];
    %[ja_deformed_x{i}, ja_deformed_y{i}] = filter_JA(traj, lambda, [], ja_fixed_points);
    
    %ax1 = subplot(6,5, i);
    %plot(ax1, [pos_x_data, pos_y_data, 'b'; dmp_deformed_x, dmp_deformed_y, 'c'; ...
    %    lte_deformed_x, lte_deformed_y, 'g--'; ja_deformed_x(:, 1), ja_deformed_y(:, 1), 'r']);
end

figure();
for i = 1:5%length(names)
    subplot(6,5, i);
    plot(pos_x_data{i}, pos_y_data{i}, 'b', dmp_deformed_x{i}, dmp_deformed_y{i}, ...
        'c', lte_deformed_x{i}, lte_deformed_y{i}, 'g', ja_deformed_x{i}(:, 1), ...
        ja_deformed_y{i}(:, 1), 'r');
    %hold on;
    %title(names{i});
    %input('press enter to continue');
    %hold off;
end