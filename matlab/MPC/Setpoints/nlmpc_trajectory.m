function [trajectory] = nlmpc_trajectory(nx, ny, nu, setting_time, p, c, robot, position)
%NLMPC_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

nlobj = nlmpc(nx,ny,nu);

%% MPC Parameters
Ts = setting_time/p;

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

%% Model Functions
nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

%% Model Weights
nlobj.Weights.ManipulatedVariables = [zeros(1, 6)];
nlobj.Weights.OutputVariables = [ones(1, 8), zeros(1, 6)];
nlobj.Weights.ManipulatedVariablesRate = [0.2*ones(1, 6)];

%% Constraints
for ct = 1:nu
    nlobj.MV(ct).Min = -pi;
    nlobj.MV(ct).Max =  pi;
end

% for ct = 8:ny
%     nlobj.OV(ct).Min = -pi;
%     nlobj.OV(ct).Max =  pi;
% end

% Joint Position Constraints
% 1
nlobj.OV(9).Min =  -2.7925;     %-160 degrees
nlobj.OV(9).Max =   2.7925; 	%160 degrees
% 2
nlobj.OV(10).Min = -2.0944;     %-120 degrees
nlobj.OV(10).Max =  2.0944;     %120 degrees
% 3
nlobj.OV(11).Min =  -2.7925;    %-160 degrees
nlobj.OV(11).Max =  -0.331613;	%-19 degrees
% 4
nlobj.OV(12).Min = -2.7925;     %-160 degrees
nlobj.OV(12).Max =  2.7925;     %160 degrees
% 5
nlobj.OV(13).Min = -2.0944;     %-120 degrees
nlobj.OV(13).Max =  2.0944;     %120 degrees
% 6
nlobj.OV(14).Min = -2*pi;
nlobj.OV(14).Max =  2*pi;

%% Robot definition, by DQ Robotics
robot = DQ_DENSO;

theta = [0, 0, 0, 0, 0, 0]';
xm = robot.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);

%% Computing Setpoint Trajectory

phi = pi;
%phi = 0;

n_vec = [0, 1, 0];

% We need to ensure unit vector
n_vec = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

r = cos(phi/2) + sin(phi/2)*n;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

setpoint = [vec8(xd); zeros(6, 1)];

dtheta = zeros(6, 1);

mv = zeros(1,6);

options = nlmpcmoveopt;

[mv, options, info] = nlmpcmove(nlobj, x0, dtheta, setpoint', [], options);

for k = 1:nlobj.PredictionHorizon
    setpoint(:, 1 + k) = setpoint(:, 1);
end

trajectory = info.Xopt;
t = [0:Ts:setting_time];

%% Plot Trajectory

trajectory_fig = figure('Name', 'MPC Generated Trajectory')

subplot(4, 2, 1);
plot(t, trajectory(:, 1));
hold on
plot(t, setpoint(1, :));
xlabel("t(s)");
legend('x_{m_1}', 'x_{d_1}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 2);
plot(t, trajectory(:, 2));
hold on
plot(t, setpoint(2, :));
xlabel("t(s)");
legend('x_{m_2}', 'x_{d_2}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 3);
plot(t, trajectory(:, 3));
hold on
plot(t, setpoint(3, :));
xlabel("t(s)");
legend('x_{m_3}', 'x_{d_3}');
xlim([0, t(end)]);
ylim([-2, 2]);
hold off

subplot(4, 2, 4);
plot(t, trajectory(:, 4));
hold on
plot(t, setpoint(4, :));
xlabel("t(s)");
legend('x_{m_4}', 'x_{d_4}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 5);
plot(t, trajectory(:, 5));
hold on
plot(t, setpoint(5, :));
xlabel("t(s)");
legend('x_{m_5}', 'x_{d_5}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 6);
plot(t, trajectory(:, 6));
hold on
plot(t, setpoint(6, :));
xlabel("t(s)");
legend('x_{m_6}', 'x_{d_6}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 7);
plot(t, trajectory(:, 7));
hold on
plot(t, setpoint(7, :));
xlabel("t(s)");
legend('x_{m_7}', 'x_{d_7}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 8);
plot(t, trajectory(:, 8));
hold on
plot(t, setpoint(8, :));
xlabel("t(s)");
legend('x_{m_8}', 'x_{d_8}');
xlim([0, t(end)]);
hold off

end

