%%% get hd and fd from data in an h5 file %%%

filename = 'h5 files/hello2.h5_grid10.h5';

grid_size = 10;

for i = 1:grid_size
    for j = 1:grid_size
        python_i = i - 1;
        python_j = j - 1;
        %disp(['(' num2str(python_i) ', ' num2str(python_j) ')/x']);
        org_x{i, j} = h5read(filename, ['/hello/original/(' num2str(python_i) ', ' num2str(python_j) ')/x']);
        org_y{i, j} = h5read(filename, ['/hello/original/(' num2str(python_i) ', ' num2str(python_j) ')/y']);
        dmp_x{i, j} = h5read(filename, ['/hello/dmp/(' num2str(python_i) ', ' num2str(python_j) ')/x']);
        dmp_y{i, j} = h5read(filename, ['/hello/dmp/(' num2str(python_i) ', ' num2str(python_j) ')/y']);
        dmp_x{i, j} = preprocess_traj(dmp_x{i, j}, 1000);
        dmp_y{i, j} = preprocess_traj(dmp_y{i, j}, 1000);
        ja_x{i, j}  = h5read(filename, ['/hello/ja/(' num2str(python_i) ', ' num2str(python_j) ')/x']);
        ja_y{i, j}  = h5read(filename, ['/hello/ja/(' num2str(python_i) ', ' num2str(python_j) ')/y']);
        lte_x{i, j} = h5read(filename, ['/hello/lte/(' num2str(python_i) ', ' num2str(python_j) ')/x']);
        lte_y{i, j} = h5read(filename, ['/hello/lte/(' num2str(python_i) ', ' num2str(python_j) ')/y']);
        [dmp_hd{i, j}, dmp_fd{i, j}] = get_dists(org_x{i, j}, org_y{i, j}, dmp_x{i, j}, dmp_y{i, j});
        [ja_hd{i, j}, ja_fd{i, j}] = get_dists(org_x{i, j}, org_y{i, j}, ja_x{i, j}, ja_y{i, j});
        [lte_hd{i, j}, lte_fd{i, j}] = get_dists(org_x{i, j}, org_y{i, j}, lte_x{i, j}, lte_y{i, j});
        
    end
end

graph_gradient(grid_size, dmp_fd, dmp_hd, 'DMP')
graph_gradient(grid_size, ja_fd, ja_hd, 'JA')
graph_gradient(grid_size, lte_fd, lte_hd, 'LTE')