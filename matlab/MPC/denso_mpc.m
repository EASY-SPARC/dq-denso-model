%% denso dual quaternion representation controlled by Adaptive MPC
close all;
clear all;
clear classes;
clc;

%% Create a new DQ_kinematics object with standard Denavit-Hartenberg parameters           
denso = DQ_DENSO;

theta = [0, 0, -pi/2, 0, 0, 0]';
xm = denso.fkm(theta);

figure;
axis equal;
plot(denso, theta, 'nobase');

grid off;
view(-0,0)
hold on;

plot(denso, theta);

%% Computing initial plant model
% State-space matrices derived from the linearization model equations
A = zeros(14, 14);
B = [denso.jacobian(theta); eye(6)];
C = eye(14);
D = zeros(14, 6);
sys = ss(A,B,C,D); % State-space model.

% Nominal values of x, y and u
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';  % According to h10 and h20
y0 = C*x0 + D*u0;

%Discretize the plant model for MPC controller
Ts = 0.01;
plant = c2d(sys,Ts);

% Specify signal types used in MPC
plant.InputGroup.ManipulatedVariables = 6;  % drad: Angular position derivative is used as input
plant.OutputGroup.Measured = 14;
plant.InputName = {'dtheta1', 'dtheta2', 'dtheta3', 'dtheta4', 'dtheta5', 'dtheta6'};
%plant.InputUnit = {'drad'};
plant.OutputName = {'a1', 'b1', 'c1', 'd1', 'a2', 'b2', 'c2', 'd2', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'dtheta6'};
%plant.OutputUnit = {'cm','cm'};

%% Setting MPC Controller
% Create MPC controller
mpcobj = mpc(plant);
% Specify prediction horizon
mpcobj.PredictionHorizon = 20;
% Specify control horizon
mpcobj.ControlHorizon = 2;

% Set nominal values in the controller
mpcobj.Model.Nominal = struct('X', x0, 'U', u0, 'Y', y0, 'DX', plant.A*x0 + plant.B*u0 - x0);

% Set scale factors
Uscale = pi * ones(1, 6);
Yscale = ones(1, 14);
for k = 1:6
    mpcobj.MV(k).ScaleFactor = Uscale(k);
end
for k = 1:14
    mpcobj.OV(k).ScaleFactor = Yscale(k);
end

% Set weights so that x is controlled and theta can float
mpcobj.Weights.OV = [ones(1, 8), zeros(1, 6)];

%mpcobj.Weights.ManipulatedVariablesRate = [0.4, 0.4, 0.4, 0.4, 0.4];

% Specify Physical constraints for OV
for k = 1:6
    mpcobj.MV(k).Min = -pi/6;
    mpcobj.MV(k).Max = pi/6;
end
for k = 1:8
    mpcobj.OV(k).Min = -Inf;
    mpcobj.OV(k).Max = Inf;
end
for k = 9:14
    mpcobj.OV(k).Min = -pi/2;
    mpcobj.OV(k).Max = pi/2;
end

%% Defining setpoint
% position = [16, 0, 7];
% position = [15, 0, 2];
position = [0.4, 0, 0.2];

phi = pi;
%phi = 0;

n_vec = [0, 1, 0];
% We need to ensure unit vector
n_vec = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

r = cos(phi/2) + sin(phi/2)*n;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

%% Computing setpoint trajectory
setting_time = 3;

setpoint = zeros(14, setting_time/Ts); 
for k = 1:setting_time/Ts
    setpoint(:, k) = [vec8(trajectory(xm, xd, k/(setting_time/Ts))); zeros(6, 1)];% theta contribution will be irrelevant because of zero weights
end

for k = 1:mpcobj.PredictionHorizon*2
    setpoint(:, setting_time/Ts + k) = setpoint(:, setting_time/Ts);
end

%% Plotting setpoints

%for k = 1:size(setpoint,2)
%    plot(DQ(setpoint(1:8, k)));
%    hold on
%end

%% Closed loop control law
epsilon = 0.015; % The error tolerance
%epsilon = 0.005;
error = setpoint(1:8, 1) - vec8(xm);

data = [vec8(xm)', theta'];

setEstimator(mpcobj, 'custom');

dtheta = zeros(6, 1);

k = 1;
%while (norm(error) > epsilon)
while (k < setting_time/Ts)
    B = [denso.jacobian(theta); eye(6)]; % Only change jacobian based on theta

    sys = ss(A,B,C,D); % A, C and D remains the same

    plant = c2d(sys,Ts);

    % Specify signal types used in MPC
    plant.InputGroup.ManipulatedVariables = 6;  % drad: Angular position derivative is used as input
    plant.OutputGroup.Measured = 14;
    plant.InputName = {'dtheta1', 'dtheta2', 'dtheta3', 'dtheta4', 'dtheta5', 'dtheta6'};
    %plant.InputUnit = {'drad'};
    plant.OutputName = {'a1', 'b1', 'c1', 'd1', 'a2', 'b2', 'c2', 'd2', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'dtheta6'};
    %plant.OutputUnit = {'cm','cm'}

    x = mpcstate(mpcobj);
    x.Plant = [vec8(denso.fkm(theta)); theta];

    %nominal = struct('X', x.Plant, 'U', dtheta*Ts, 'Y', x.Plant, 'DX', B*dtheta*Ts);
    nominal = struct('X', x.Plant, 'U', zeros(6, 1), 'Y', x.Plant, 'DX', zeros(14, 1));

    [dtheta, info] = mpcmoveAdaptive(mpcobj, x, plant, nominal, [], setpoint(:, k:k+mpcobj.PredictionHorizon)', []);

    theta = theta + dtheta * Ts;

    error = setpoint(1:8, k) - x.Plant(1:8);

    plot(denso, theta');
    drawnow;
    
    if (k<size(setpoint,2))
        k = k+1;
    end
    
    data = [data; x.Plant'];
end

%% Plotting performance
t = Ts:Ts:k*Ts;

if k < size(setpoint, 2)
    setpoint = setpoint(:, 1:k);
elseif k > size(setpoint, 2)
    for i = 1:k-size(setpoint, 2)
        setpoint(:, i) = setpoint(:, end);
    end
end

subplot(4, 2, 1);
plot(t, data(:, 1));
hold on
plot(t, setpoint(1, :));
xlabel("t(s)");
legend('x_{m_1}', 'x_{d_1}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 2);
plot(t, data(:, 2));
hold on
plot(t, setpoint(2, :));
xlabel("t(s)");
legend('x_{m_2}', 'x_{d_2}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 3);
plot(t, data(:, 3));
hold on
plot(t, setpoint(3, :));
xlabel("t(s)");
legend('x_{m_3}', 'x_{d_3}');
xlim([0, t(end)]);
ylim([-2, 2]);
hold off

subplot(4, 2, 4);
plot(t, data(:, 4));
hold on
plot(t, setpoint(4, :));
xlabel("t(s)");
legend('x_{m_4}', 'x_{d_4}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 5);
plot(t, data(:, 5));
hold on
plot(t, setpoint(5, :));
xlabel("t(s)");
legend('x_{m_5}', 'x_{d_5}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 6);
plot(t, data(:, 6));
hold on
plot(t, setpoint(6, :));
xlabel("t(s)");
legend('x_{m_6}', 'x_{d_6}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 7);
plot(t, data(:, 7));
hold on
plot(t, setpoint(7, :));
xlabel("t(s)");
legend('x_{m_7}', 'x_{d_7}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 8);
plot(t, data(:, 8));
hold on
plot(t, setpoint(8, :));
xlabel("t(s)");
legend('x_{m_8}', 'x_{d_8}');
xlim([0, t(end)]);
hold off