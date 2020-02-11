function [hd, fd] = get_dists(x1, y1, x2, y2)
traj1 = [x1, y1];
[traj1_rows, traj1_cols] = size(traj1);
if (traj1_rows == 1)
    traj1 = [x1; y1];
end
if (traj1_cols > traj1_rows)
    traj1 = traj1';
end
traj2 = [x2, y2];
[traj2_rows, traj2_cols] = size(traj2);
if (traj2_rows == 1)
    traj2 = [x2; y2];
end
if (traj2_cols > traj2_rows)
    traj2 = traj2';
end
%disp(size(traj1));
%disp(size(traj2));
%pause;
[fd, ~] = DiscreteFrechetDist(traj1, traj2);
hd = HausdorffDist(traj1, traj2);
end