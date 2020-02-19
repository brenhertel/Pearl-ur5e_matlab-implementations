function graph_gradient_simple(grid_size, fd, hd, name)
%%% gradient representation of similarity metrics %%%
img_increment = round(500 / grid_size);
imgfd = zeros(grid_size * img_increment, grid_size * img_increment);
imghd = zeros(grid_size * img_increment, grid_size * img_increment);
%min_fd = min(min(cell2mat(fd)));
%max_fd = max(max(cell2mat(fd)));
%min_hd = min(min(cell2mat(hd)));
%max_hd = max(max(cell2mat(hd)));
for i = 1:grid_size
    for j = 1:grid_size
        for a = 1:img_increment
            for b = 1:img_increment
                imgfd(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b) = map(fd(i, j), 0, 1, 0, 255);
                imghd(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b) = map(hd(i, j), 0, 1, 0, 255);
            end
        end
    end
end
figure;
imgfd = uint8(imgfd);
imshow(imgfd);
title([name ' Frechet Distance Gradient'], 'FontSize', 28);
figure;
imghd = uint8(imghd);
imshow(imghd);
title([name ' Hausdorff Distance Gradient'], 'FontSize', 28);
end