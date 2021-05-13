function H = hemi4minus(u)
%TESTE Summary of this function goes here
%   Detailed explanation goes here
H = u(1)*eye(4);

H(1,2) = -u(2);
H(1,3) = -u(3);
H(1,4) = -u(4);

H(2,1) = u(2);
H(2,3) = u(4);
H(2,4) = -u(3);

H(3,1) = u(3);
H(3,2) = -u(4);
H(3,4) = u(2);

H(4,1) = u(4);
H(4,2) = u(3);
H(4,3) = -u(2);

end

