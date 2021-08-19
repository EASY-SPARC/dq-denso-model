function H = hemi8plus(u)
%HEMI_PLUS Summary of this function goes here
%   Detailed explanation goes here
H = [   hemi4plus(u(1:4)),   zeros(4,4);
        hemi4plus(u(5:end)), hemi4plus(u(1:4))];

end

