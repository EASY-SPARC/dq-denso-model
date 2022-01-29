nx = 14;
ny = 14;
nu = 6;
nlobj = nlmpc(nx,ny,nu);

%% MPC Parameters
Ts = .1;
p = 10;
c = 10;
setting_time = 10;

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

%% Model Functions
nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

%% Model Weights
nlobj.Weights.ManipulatedVariables = [zeros(1, 6)];
nlobj.Weights.OutputVariables = [ones(1, 4), 10*ones(1, 2), 10*ones(1, 2), zeros(1, 6)];
nlobj.Weights.ManipulatedVariablesRate = [ones(1, 6)];

%% Constraints
for ct = 1:nu
    nlobj.MV(ct).Min = -pi;
    nlobj.MV(ct).Max =  pi;
end
 
% for ct = 9:ny
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

%% Denso definition, by DQ Robotics
denso = DQ_DENSO;

theta = [0, 0, 0, 0, 0, 0]';
xm = denso.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);

%% Computing Setpoint Trajectory
% trajectory_length = setting_time/Ts;
% setpoint = nlmpc_trajectory(14,14,6,setting_time,trajectory_length,trajectory_length,denso,[0.2,0,0.2])';
path = [0.2*ones(setting_time/Ts, 1) zeros(setting_time/Ts, 1) 0.2*ones(setting_time/Ts, 1)];
setpoint = path_to_dq(path, pi/2);

for k = 1:nlobj.PredictionHorizon*2
    setpoint(:, length(setpoint) + 1) = setpoint(:, length(setpoint));
end

%% Closed loop control law
x = x0;
data = zeros(14, trajectory_length);
mv_data = zeros(6, trajectory_length);
dtheta = zeros(6, 1);
mv = zeros(1,6);

options = nlmpcmoveopt;

tic;
hbar = waitbar(0,'Simulation Progress');
robot_plot = figure('Name', 'Robot Plot');

k = 1;
while (k <= setting_time/Ts)
    
    [mv, options, info] = nlmpcmove(nlobj, x, dtheta, setpoint(:, k+1:k+nlobj.PredictionHorizon)', [], options);
    
    
    dtheta = info.MVopt(1,:)';
    theta = theta + dtheta * Ts;
    
    xm = denso.fkm(theta);
    
    x = [vec8(xm); theta];
    
    plot(denso, theta');
    drawnow;
    
    data(:, k) = x;
    mv_data(:, k) = dtheta';
    
    waitbar(k/(setting_time/Ts), hbar);
    k = k+1;
end

timeElapsed = toc;
close(hbar);
data = [x0 data];
mv_data = [zeros(6,1) mv_data];

%% Plotting performance
t = 0:Ts:(k-1)*Ts;

if k < size(setpoint, 2)
    setpoint = setpoint(:, 1:k);
elseif k > size(setpoint, 2)
    for i = 1:k-size(setpoint, 2)
        setpoint(:, i) = setpoint(:, end);
    end
end

drawnow;

%% Trajectory Following Performance
output_results = figure('Name', 'Trajectory Following');

subplot(4, 2, 1);
plot(t, data(1, :));
hold on
plot(t, setpoint(1, :));
xlabel("t(s)");
legend('x_{m_1}', 'x_{d_1}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 2);
plot(t, data(2, :));
hold on
plot(t, setpoint(2, :));
xlabel("t(s)");
legend('x_{m_2}', 'x_{d_2}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 3);
plot(t, data(3, :));
hold on
plot(t, setpoint(3, :));
xlabel("t(s)");
legend('x_{m_3}', 'x_{d_3}');
xlim([0, t(end)]);
ylim([-2, 2]);
hold off

subplot(4, 2, 4);
plot(t, data(4, :));
hold on
plot(t, setpoint(4, :));
xlabel("t(s)");
legend('x_{m_4}', 'x_{d_4}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 5);
plot(t, data(5, :));
hold on
plot(t, setpoint(5, :));
xlabel("t(s)");
legend('x_{m_5}', 'x_{d_5}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 6);
plot(t, data(6, :));
hold on
plot(t, setpoint(6, :));
xlabel("t(s)");
legend('x_{m_6}', 'x_{d_6}');
xlim([0, t(end)]);
hold off

subplot(4, 2, 7);
plot(t, data(7, :));
hold on
plot(t, setpoint(7, :));
xlabel("t(s)");
legend('x_{m_7}', 'x_{d_7}');
xlim([0, t(end)]);
ylim([-.005, .005]);
hold off

subplot(4, 2, 8);
plot(t, data(8, :));
hold on
plot(t, setpoint(8, :));
xlabel("t(s)");
legend('x_{m_8}', 'x_{d_8}');
xlim([0, t(end)]);
hold off

%% Manipulated Variables Behaviour
mv_results = figure('Name', 'Manipulated Variables');

subplot(3, 2, 1);
plot(t, mv_data(1, :));
xlabel("t(s)");
legend('dx_{m_9}');
xlim([0, t(end)]);

subplot(3, 2, 2);
plot(t, mv_data(2, :));
xlabel("t(s)");
legend('dx_{m_{10}}');
xlim([0, t(end)]);

subplot(3, 2, 3);
plot(t, mv_data(3, :));
xlabel("t(s)");
legend('dx_{m_{11}}');
xlim([0, t(end)]);

subplot(3, 2, 4);
plot(t, mv_data(4, :));
xlabel("t(s)");
legend('dx_{m_{12}}');
xlim([0, t(end)]);

subplot(3, 2, 5);
plot(t, mv_data(5, :));
xlabel("t(s)");
legend('dx_{m_{13}}');
xlim([0, t(end)]);

subplot(3, 2, 6);
plot(t, mv_data(6, :));
xlabel("t(s)");
legend('dx_{m_{14}}');
xlim([0, t(end)]);