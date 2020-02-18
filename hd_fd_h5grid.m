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

dmp_fd_mat = normalized(dmp_fd);
dmp_hd_mat = normalized(dmp_hd);
lte_fd_mat = normalized(lte_fd);
lte_hd_mat = normalized(lte_hd);
ja_fd_mat = normalized(ja_fd);
ja_hd_mat = normalized(ja_hd);

for i = 1:grid_size
        x(i) = lte_x{1, i}(1);
        y(i) = lte_y{i, 1}(1);
end
% 
% [X, Y] = meshgrid(x, y);
% figure;
% surf(X, Y, dmp_fd_mat);
% title('DMP FD Surface');
% figure;
% surf(X, Y, dmp_hd_mat);
% title('DMP HD Surface');
% figure;
% surf(X, Y, lte_fd_mat);
% title('LTE FD Surface');
% figure;
% surf(X, Y, lte_hd_mat);
% title('LTE HD Surface');
% figure;
% surf(X, Y, ja_fd_mat);
% title('JA FD Surface');
% figure;
% surf(X, Y, ja_hd_mat);
% title('JA HD Surface');
n = 1;
for i = 1:grid_size
    for j = 1:grid_size
        coords_dmp_fd(n, 1:3) = [x(i), y(j), dmp_fd_mat(i, j)];
        coords_dmp_hd(n, 1:3) = [x(i), y(j), dmp_hd_mat(i, j)];
        coords_lte_fd(n, 1:3) = [x(i), y(j), lte_fd_mat(i, j)];
        coords_lte_hd(n, 1:3) = [x(i), y(j), lte_hd_mat(i, j)];
        coords_ja_fd(n, 1:3) = [x(i), y(j), ja_fd_mat(i, j)];
        coords_ja_hd(n, 1:3) = [x(i), y(j), ja_hd_mat(i, j)];
        n = n + 1;
    end
end
figure;
fitobj_dmp_fd = fit([coords_dmp_fd(:, 1), coords_dmp_fd(:, 2)], coords_dmp_fd(:, 3), 'lowess');
plot(fitobj_dmp_fd,[coords_dmp_fd(:, 1), coords_dmp_fd(:, 2)],coords_dmp_fd(:, 3))
title('DMP FD lowess Surface');
figure;
fitobj_dmp_hd = fit([coords_dmp_hd(:, 1), coords_dmp_hd(:, 2)], coords_dmp_hd(:, 3), 'lowess');
plot(fitobj_dmp_hd,[coords_dmp_hd(:, 1), coords_dmp_hd(:, 2)],coords_dmp_hd(:, 3))
title('DMP HD lowess Surface');
figure;
fitobj_lte_fd = fit([coords_lte_fd(:, 1), coords_lte_fd(:, 2)], coords_lte_fd(:, 3), 'lowess');
plot(fitobj_lte_fd,[coords_lte_fd(:, 1), coords_lte_fd(:, 2)],coords_lte_fd(:, 3))
title('LTE FD lowess Surface');
figure;
fitobj_lte_hd = fit([coords_lte_hd(:, 1), coords_lte_hd(:, 2)], coords_lte_hd(:, 3), 'lowess');
plot(fitobj_lte_hd,[coords_lte_hd(:, 1), coords_lte_hd(:, 2)],coords_lte_hd(:, 3))
title('LTE HD lowess Surface');
figure;
fitobj_ja_fd = fit([coords_ja_fd(:, 1), coords_ja_fd(:, 2)], coords_ja_fd(:, 3), 'lowess');
plot(fitobj_ja_fd,[coords_ja_fd(:, 1), coords_ja_fd(:, 2)],coords_ja_fd(:, 3))
title('JA FD lowess Surface');
figure;
fitobj_ja_hd = fit([coords_ja_hd(:, 1), coords_ja_hd(:, 2)], coords_ja_hd(:, 3), 'lowess');
plot(fitobj_ja_hd,[coords_ja_hd(:, 1), coords_ja_hd(:, 2)],coords_ja_hd(:, 3))
title('JA HD lowess Surface');

% graph_gradient(grid_size, dmp_fd, dmp_hd, 'DMP')
% graph_gradient(grid_size, ja_fd, ja_hd, 'JA')
% graph_gradient(grid_size, lte_fd, lte_hd, 'LTE')