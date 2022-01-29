function [trajectory, vecxd] = nlmpc_trajectory(nx, ny, nu, setting_time, p, c, robot, center, r, phi)
%NLMPC_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

nlobj = nlmpc(nx,ny,nu);

%% MPC Parameters
Ts = setting_time/p;

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p/10;
nlobj.ControlHorizon = c/10;

%% Model Functions
nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

%% Model Weights
nlobj.Weights.OutputVariables = [10, 100, 10, 100, 200, 200, 10, 50, zeros(1, 6)];
nlobj.Weights.ManipulatedVariablesRate = [ones(1, 6)];

%% Constraints
for ct = 1:nu
    nlobj.MV(ct).Min = -pi;
    nlobj.MV(ct).Max =  pi;
end

for ct = 8:ny
    nlobj.OV(ct).Min = -pi;
    nlobj.OV(ct).Max =  pi;
end

%% Robot definition, by DQ Robotics
robot = DQ_DENSO;

theta = [0, 0, 0, 0, 0, 0]';
xm = robot.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);

%% Computing Setpoint Trajectory

[x_circle, y_circle, z_circle] = nlmpc_circle_path(center,r,0,setting_time,Ts,0,4*pi);
[setpoint, vecxd] = path_to_dq([x_circle, y_circle, z_circle], phi);

dtheta = zeros(6, 1);

mv = zeros(1,6);

options = nlmpcmoveopt;

trajectory = zeros(p+1, nx);
for k = 1:10
    [mv, options, info] = nlmpcmove(nlobj, x0, dtheta, setpoint(:, 10*(k-1)+1:10*k)', [], options);
    x0 = info.Xopt(end, :);
    dtheta = info.MVopt(end,:)';
    trajectory(10*(k-1)+1:10*k, :) = info.Xopt(1:end-1,:);
end
trajectory(end,:) = info.Xopt(end,:);
    
t = [0:Ts:setting_time];

%% Plot Trajectory

trajectory_fig = figure('Name', 'MPC Generated Trajectory');

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

