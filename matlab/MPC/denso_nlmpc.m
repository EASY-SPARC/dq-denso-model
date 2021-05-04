nx = 13;
ny = 13;
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
nlobj.Weights.OutputVariables = [10*ones(1, 3), 1*ones(1, 4), zeros(1, 6)];
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
pm = vec4(xm.translation);
pm = pm(2:end);
rm = xm.P;
x0 = [pm; vec4(rm); theta];
u0 = [0, 0, 0, 0, 0, 0]';

% validateFcns(nlobj,x0,u0);

%% Setpoint
position = [0.1, -0.1, 0.4];
phi = pi;

n_vec = [0, 1, 0];

% We need to ensure unit vector
n_vec = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

rd = cos(phi/2) + sin(phi/2)*n;

pd = [position(1), position(2), position(3)]';

xd = [pd; vec4(rd)];

%% Computing Setpoint Trajectory

pose(:,1) = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';
pose(:,2) = [-0.15204131291488196, 0.03551023872742621, -0.009468388393512543, 3.442978934808796e-06, -0.02603743293975172, 0.15204180668445277]';
pose(:,3) = [-0.3040826258297639, 0.07102047745485242, -0.018936776787025086, 6.885957869617592e-06, -0.05207486587950344, 0.30408361336890555]';
pose(:,4) = [-0.4561239387446459, 0.10653071618227862, -0.02840516518053763, 1.0328936804426387e-05, -0.07811229881925516, 0.4561254200533583]';
pose(:,5) = [-0.6081652516595278, 0.14204095490970484, -0.03787355357405017, 1.3771915739235184e-05, -0.10414973175900688, 0.6081672267378111]';
pose(:,6) = [-0.7602065645744099, 0.17755119363713104, -0.04734194196756272, 1.721489467404398e-05, -0.13018716469875863, 0.760209033422264]';
pose(:,7) = [-0.9122478774892918, 0.21306143236455724, -0.05681033036107526, 2.0657873608852775e-05, -0.15622459763851032, 0.9122508401067166]';
pose(:,8) = [-1.0642891904041738, 0.24857167109198347, -0.06627871875458781, 2.4100852543661575e-05, -0.18226203057826207, 1.0642926467911695]';
pose(:,9) = [-1.2163305033190557, 0.2840819098194097, -0.07574710714810035, 2.754383147847037e-05, -0.20829946351801376, 1.2163344534756222]';
pose(:,10) = [-1.3683718162339378, 0.3195921485468359, -0.0852154955416129, 3.0986810413279166e-05, -0.2343368964577655, 1.368376260160075]';
pose(:,11) = [-1.5204131291488199, 0.3551023872742621, -0.09468388393512545, 3.442978934808796e-05, -0.26037432939751726, 1.520418066844528]';
pose(:,12) = [-1.6724544420637018, 0.39061262600168833, -0.104152272328638, 3.787276828289676e-05, -0.286411762337269, 1.6724598735289808]';
pose(:,13) = [-1.8244957549785836, 0.4261228647291145, -0.11362066072215052, 4.131574721770555e-05, -0.31244919527702064, 1.8245016802134333]';
pose(:,14) = [-1.9765370678934657, 0.4616331034565407, -0.12308904911566307, 4.475872615251435e-05, -0.3384866282167724, 1.9765434868978862]';
pose(:,15) = [-2.1285783808083476, 0.49714334218396694, -0.13255743750917562, 4.820170508732315e-05, -0.36452406115652414, 2.128585293582339]';
pose(:,16) = [-2.2806196937232297, 0.5326535809113931, -0.14202582590268817, 5.164468402213195e-05, -0.3905614940962759, 2.2806271002667917]';
pose(:,17) = [-2.4326610066381114, 0.5681638196388193, -0.1514942142962007, 5.508766295694074e-05, -0.4165989270360275, 2.4326689069512444]';
pose(:,18) = [-2.5847023195529935, 0.6036740583662455, -0.16096260268971324, 5.8530641891749535e-05, -0.44263635997577927, 2.5847107136356975]';
pose(:,19) = [-2.7367436324678756, 0.6391842970936717, -0.1704309910832258, 6.197362082655833e-05, -0.468673792915531, 2.73675252032015]';

setting_time = (length(pose)-1)*Ts;
setpoint = zeros(13, length(pose));

for itter_pose = 1:length(pose)
   pose_fkm = denso.fkm(pose(:,itter_pose));
   
   pose_pm = vec4(pose_fkm.translation);
   setpoint(1:3,itter_pose) = pose_pm(2:end);
   
   setpoint(4:7,itter_pose) = vec4(pose_fkm.P);
end

% for k = 1:setting_time/Ts
%     setpoint(:, k) = [trajectory_pos(pm, pd, k/(setting_time/Ts)); vec4(trajectory(rm, rd, k/(setting_time/Ts))); zeros(6, 1)];% theta contribution will be irrelevant because of zero weights
% %     setpoint(:, k) = [xd; zeros(6, 1)];
% end

for k = 1:nlobj.PredictionHorizon*2
    setpoint(:, length(pose) + k) = setpoint(:, length(pose));
%     setpoint(:, setting_time/Ts + k) = [xd; zeros(6, 1)];
end

%% Closed loop control law

x = x0;
data = [pm', vec4(rm)', theta'];

dtheta = zeros(6, 1);

k = 1;

mv = zeros(1,6);

options = nlmpcmoveopt;

% tic
hbar = waitbar(0,'Simulation Progress');

while (k <= setting_time/Ts)
    
    [mv, options, info] = nlmpcmove(nlobj, x, dtheta, setpoint(:, k+1:k+nlobj.PredictionHorizon)', [], options);
    
    
    dtheta = info.MVopt(1,:)';
    theta = theta + dtheta * Ts;
    
    xm = denso.fkm(theta);
    pm = vec4(xm.translation);
    pm = pm(2:end);
    rm = xm.P;
    
    x = [pm; vec4(rm); theta];
    
    plot(denso, theta');
    drawnow;
    pause(.5)
    
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
hold off

subplot(4, 2, 2);
plot(t, data(:, 2));
hold on
plot(t, setpoint(2, :));
xlabel("t(s)");
legend('x_{m_2}', 'x_{d_2}');
hold off

subplot(4, 2, 3);
plot(t, data(:, 3));
hold on
plot(t, setpoint(3, :));
xlabel("t(s)");
legend('x_{m_3}', 'x_{d_3}');
hold off

subplot(4, 2, 4);
plot(t, data(:, 4));
hold on
plot(t, setpoint(4, :));
xlabel("t(s)");
legend('x_{m_4}', 'x_{d_4}');
hold off

subplot(4, 2, 5);
plot(t, data(:, 5));
hold on
plot(t, setpoint(5, :));
xlabel("t(s)");
legend('x_{m_5}', 'x_{d_5}');
hold off

subplot(4, 2, 6);
plot(t, data(:, 6));
hold on
plot(t, setpoint(6, :));
xlabel("t(s)");
legend('x_{m_6}', 'x_{d_6}');
hold off

subplot(4, 2, 7);
plot(t, data(:, 7));
hold on
plot(t, setpoint(7, :));
xlabel("t(s)");
legend('x_{m_7}', 'x_{d_7}');
hold off