function result = trajectory_pos(pos_m, pos_d, t)
%TRAJECTORY_POS Summary of this function goes here
%   Detailed explanation goes here
result = pos_m*(1-t) + pos_d*t;
end

