function H = hemi8minus(u)
%HEMI_PLUS Summary of this function goes here
%   Detailed explanation goes here
H = [   hemi4minus(u(1:4)),   zeros(4,4);
        hemi4minus(u(5:end)), hemi4minus(u(1:4))];

end

