function graph_diff2(grid_size, mat1, mat2, name)
%%% image representation of better similarity metrics %%%
img_increment = round(500 / grid_size);
img = zeros(grid_size * img_increment, grid_size * img_increment, 3);
for i = 1:grid_size
    for j = 1:grid_size
        for a = 1:img_increment
            for b = 1:img_increment
                if (mat1(i, j) > mat2(i, j))
                    img(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b, 2) = 255;
                elseif (mat2(i, j) > mat1(i, j))
                    img(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b, 1) = 255;
                else
                    img(((i - 1) * img_increment) + a, ((j - 1) * img_increment) + b, 3) = 255;                    
                end
            end
        end
    end
end
figure;
img = uint8(img);
imshow(img);
title([name ' Distance Difference Gradient'], 'FontSize', 28);
end