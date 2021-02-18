nx = 14;
ny = 14;
nu = 6;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
p = 40;
c = 5;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;

nlobj.Model.StateFcn = "densoStateFunction";
nlobj.Jacobian.StateFcn = "densoJacobianFunction";

nlobj.Weights.OutputVariables = [ones(1, 8), zeros(1, 6)];

% nloptions = nlmpcmoveopt;
% nloptions.Parameters = {Ts};
% nlobj.Model.NumberOfParameters = 1;

for ct = 1:nu
    nlobj.MV(ct).Min = -(pi/6)*10;
    nlobj.MV(ct).Max =  (pi/6)*10;
end

for ct = 9:ny
    nlobj.OV(ct).Min = -pi/2;
    nlobj.OV(ct).Max =  pi/2;
end

denso = DQ_DENSO;

theta = [0, 0, -pi/2, 0, 0, 0]';
xm = denso.fkm(theta);
x0 = [vec8(xm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

validateFcns(nlobj,x0,u0);

%% Defining setpoint
position = [0.2, 0, 0.2];

phi = pi;

n_vec = [0, 1, 0];

% We need to ensure unit vector
n_vec = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

r = cos(phi/2) + sin(phi/2)*n;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

%% Computing setpoint trajectory
setting_time = 5;

setpoint = zeros(14, setting_time/Ts); 
for k = 1:setting_time/Ts
    setpoint(:, k) = [vec8(trajectory(xm, xd, k/(setting_time/Ts))); zeros(6, 1)];% theta contribution will be irrelevant because of zero weights
end

for k = 1:nlobj.PredictionHorizon*2
    setpoint(:, setting_time/Ts + k) = setpoint(:, setting_time/Ts);
end

%% Closed loop control law

x = x0;
data = [vec8(xm)', theta'];

dtheta = zeros(6, 1);

k = 1;

options = nlmpcmoveopt;

while (k < setting_time/Ts)
    
    [~, ~, info] = nlmpcmove(nlobj, x, dtheta, setpoint(:, k:k+nlobj.PredictionHorizon)', [], options);
%     [~, ~, info] = nlmpcmove(nlobj, x, dtheta, kron(ones(nlobj.PredictionHorizon,1),setpoint(:, end)'), []);
%     dtheta = nlmpcmove(nlobj, x, dtheta, setpoint(:, k:k+nlobj.PredictionHorizon)', [], nloptions)
%     disp(setpoint(:, k:k+nlobj.PredictionHorizon)');

    dtheta = info.MVopt(1,:)'
    theta = theta + dtheta * Ts;
    
%     x = x + densoStateFunction(x, dtheta)*Ts
    x = [vec8(denso.fkm(theta)); theta];

    plot(denso, theta');
    drawnow;
    
    if (k<size(setpoint,2))
        k = k+1;
    end
    
    data = [data; x'];
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