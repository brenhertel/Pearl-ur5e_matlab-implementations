%%% Get starting grid %%%
for i = 1:grid_size
    for j = 1:grid_size
        %grid{i, j} = [lte_x{i, j}(1), lte_y{i, j}(1)];
        
        x_grid(i, j) = lte_x{i, j}(1);
        y_grid(i, j) = lte_y{i, j}(1);
    end
end
dmp_fd_mat = cell2mat(dmp_fd);
dmp_hd_mat = cell2mat(dmp_hd);
lte_fd_mat = cell2mat(lte_fd);
lte_hd_mat = cell2mat(lte_hd);
ja_fd_mat = cell2mat(ja_fd);
ja_hd_mat = cell2mat(ja_hd);

for i = 1:grid_size
    for j = 1:grid_size
        x = x_grid(i, j);
        y = y_grid(i, j);
        
    end
end

% filename = ['h5 files/xy_fd_hd_grid' num2str(grid_size) '.h5'];
% h5create(filename,'/x',[10 10]);
% h5write(filename, '/x', x_grid);
% h5create(filename,'/y',[10 10]);
% h5write(filename, '/y', y_grid);
% h5create(filename,'/dmp_fd',[10 10]);
% h5write(filename, '/dmp_fd', dmp_fd_mat);
% h5create(filename,'/lte_fd',[10 10]);
% h5write(filename, '/lte_fd', lte_fd_mat);
% h5create(filename,'/ja_fd',[10 10]);
% h5write(filename, '/ja_fd', ja_fd_mat);
% h5create(filename,'/dmp_hd',[10 10]);
% h5write(filename, '/dmp_hd', dmp_hd_mat);
% h5create(filename,'/lte_hd',[10 10]);
% h5write(filename, '/lte_hd', lte_hd_mat);
% h5create(filename,'/ja_hd',[10 10]);
% h5write(filename, '/ja_hd', ja_hd_mat);