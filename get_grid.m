%%% Get starting grid %%%
for i = 1:grid_size
    for j = 1:grid_size
        grid{i, j} = [lte_x{i, j}(1), lte_y{i, j}(1)];
    end
end