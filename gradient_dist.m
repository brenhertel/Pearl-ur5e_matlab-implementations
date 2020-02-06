%%% gradient representation of similarity metrics %%%
img_increment = 50;
imgfd = zeros(grid_size * img_increment, grid_size * img_increment);
imghd = zeros(grid_size * img_increment, grid_size * img_increment);
for i = 1:grid_size
    for j = 1:grid_size
        for a = 1:img_increment
            for b = 1:img_increment
                imgfd(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b) = 255 * (1 - fd{i, j});
                imghd(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b) = 255 * (1 - hd{i, j});
            end
        end
    end
end
figure;
imgfd = uint8(imgfd);
imshow(imgfd);
title('Frechet Distance Gradient');
figure;
imghd = uint8(imghd);
imshow(imghd);
title('Hausdorff Distance Gradient');