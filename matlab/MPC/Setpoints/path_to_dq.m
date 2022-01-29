function [setpoint, vecxd] = path_to_dq(path, phi)
%PATH_TO_DQ Summary of this function goes here
%   Detailed explanation goes here

x = path(:,1);
y = path(:,2);
z = path(:,3);

%% Computing Setpoint Trajectory
steps = length(x);
setpoint = zeros(14, steps);

% phi = pi/2;
n_vec = [0, 1, 0];

% We need to ensure unit vector
n_vec = n_vec/norm(n_vec);
n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);
r = cos(phi/2) + sin(phi/2)*n;

vecxd = [];
for k = 1:length(x)
    p = DQ([0, x(k), y(k), z(k)]);
    xd = r + 1/2 * DQ.E * p * r;

    setpoint(:,k) = [vec8(xd); zeros(6, 1)];
    vecxd = [vecxd xd];

end

