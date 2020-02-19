%%% normalize dists %%%
function [normalized_dist1, normalized_dist2, normalized_dist3] = normalized3(dists1, dists2, dists3)
% convert from cell to matrix
dist1_mat = cell2mat(dists1);
dist2_mat = cell2mat(dists2);
dist3_mat = cell2mat(dists3);
% get max of each matrix
max_d1 = max(max(dist1_mat));
max_d2 = max(max(dist2_mat));
max_d3 = max(max(dist3_mat));
%get max of maxes (absolute max)
max_d = max([max_d1, max_d2, max_d3]);
%divide each max by absolute max
dist1_mat = dist1_mat / max_d;
dist2_mat = dist2_mat / max_d;
dist3_mat = dist3_mat / max_d;
% to make higher better, return 1-normalized
normalized_dist1 = ones(size(dist1_mat)) - dist1_mat;
normalized_dist2 = ones(size(dist2_mat)) - dist2_mat;
normalized_dist3 = ones(size(dist3_mat)) - dist3_mat;
end