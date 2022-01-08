nx = 14;
ny = 14;
nu = 6;
nlobj = nlmpc(nx,ny,nu);

%% MPC Parameters
Ts = .1;
p = 10;
c = 10;

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

%% Model Functions
nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

%% Model Weights
nlobj.Weights.OutputVariables = [ones(1, 8), zeros(1, 6)];
nlobj.Weights.ManipulatedVariablesRate = [0*ones(1, 6)];

%% Constraints
for ct = 1:nu
    nlobj.MV(ct).Min = -pi;
    nlobj.MV(ct).Max =  pi;
end

for ct = 8:ny
    nlobj.OV(ct).Min = -pi;
    nlobj.OV(ct).Max =  pi;
end

%% Denso definition, by DQ Robotics
denso = DQ_DENSO;

theta = [0, 0, 0, 0, 0, 0]';
xm = denso.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);

%% Computing Setpoint Trajectory

setpoint = nlmpc_trajectory(14,14,6,5,50,50,denso,[0.2,0,0.2])'
setting_time = (length(setpoint)-1)*Ts;

for k = 1:nlobj.PredictionHorizon*2
    setpoint(:, length(setpoint) + 1) = setpoint(:, length(setpoint));
end

%% Closed loop control law

x = x0;
data = [vec8(xm)', theta'];

dtheta = zeros(6, 1);

k = 1;

mv = zeros(1,6);

options = nlmpcmoveopt;

% tic
hbar = waitbar(0,'Simulation Progress');

storage_thetas = [theta];

while (k <= setting_time/Ts)
    
    [mv, options, info] = nlmpcmove(nlobj, x, dtheta, setpoint(:, k+1:k+nlobj.PredictionHorizon)', [], options);
    
    
    dtheta = info.MVopt(1,:)';
    theta = theta + dtheta * Ts;
    
    storage_thetas = [storage_thetas theta];
    
    xm = denso.fkm(theta);
    
    x = [vec8(xm); theta];
    
    plot(denso, theta');
    drawnow;
    
    k = k+1;
    
    data = [data; x'];
    
    waitbar(k/(setting_time/Ts), hbar);
end

close(hbar);
% timeElapsed = toc
%% Plotting performance
t = 0:Ts:(k-1)*Ts;

if k < size(setpoint, 2)
    setpoint = setpoint(:, 1:k);
elseif k > size(setpoint, 2)
    for i = 1:k-size(setpoint, 2)
        setpoint(:, i) = setpoint(:, end);
    end
end

% setpoint = [x0 setpoint];
drawnow;

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