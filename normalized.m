%%% normalize dists %%%
function normalized_dist = normalized(dist_cell)
dist_mat = cell2mat(dist_cell);
max_d = max(max(dist_mat));
dist_mat = dist_mat / max_d;
normalized_dist = ones(size(dist_mat)) - dist_mat;
end