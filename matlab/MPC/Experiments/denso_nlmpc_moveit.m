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
nlobj.Weights.OutputVariables = [5*ones(1, 8), zeros(1, 6)];
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

pose(:,1) = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';
pose(:,2) = [-0.018133084350372416, -0.036682946500843486, -0.006856627289031084, -0.13255466042976297, 0.04244720721182586, 0.13858629362332114]';
pose(:,3) = [-0.03626616870074483, -0.07336589300168697, -0.013713254578062169, -0.26510932085952593, 0.08489441442365173, 0.2771725872466423]';
pose(:,4) = [-0.05439925305111724, -0.11004883950253044, -0.02056988186709325, -0.39766398128928887, 0.12734162163547758, 0.4157588808699634]';
pose(:,5) = [-0.07253233740148966, -0.14673178600337394, -0.027426509156124337, -0.5302186417190519, 0.16978882884730345, 0.5543451744932846]';
pose(:,6) = [-0.09066542175186207, -0.18341473250421741, -0.03428313644515542, -0.6627733021488148, 0.2122360360591293, 0.6929314681166057]';
pose(:,7) = [-0.10879850610223447, -0.2200976790050609, -0.0411397637341865, -0.7953279625785777, 0.25468324327095515, 0.8315177617399268]';
pose(:,8) = [-0.12693159045260688, -0.25678062550590436, -0.04799639102321758, -0.9278826230083407, 0.29713045048278103, 0.9701040553632478]';
pose(:,9) = [-0.14506467480297933, -0.2934635720067479, -0.054853018312248675, -1.0604372834381037, 0.3395776576946069, 1.1086903489865692]';
pose(:,10) = [-0.16319775915335172, -0.33014651850759136, -0.061709645601279756, -1.1929919438678667, 0.3820248649064327, 1.2472766426098902]';
pose(:,11) = [-0.18133084350372414, -0.36682946500843483, -0.06856627289031084, -1.3255466042976296, 0.4244720721182586, 1.3858629362332113]';
pose(:,12) = [-0.19946392785409656, -0.40351241150927836, -0.07542290017934193, -1.4581012647273928, 0.4669192793300845, 1.5244492298565326]';
pose(:,13) = [-0.21759701220446895, -0.4401953580101218, -0.082279527468373, -1.5906559251571555, 0.5093664865419103, 1.6630355234798535]';
pose(:,14) = [-0.2357300965548414, -0.4768783045109653, -0.08913615475740409, -1.7232105855869186, 0.5518136937537362, 1.8016218171031748]';
pose(:,15) = [-0.25386318090521376, -0.5135612510118087, -0.09599278204643516, -1.8557652460166814, 0.5942609009655621, 1.9402081107264957]';
pose(:,16) = [-0.2719962652555862, -0.5502441975126522, -0.10284940933546625, -1.9883199064464443, 0.6367081081773879, 2.078794404349817]';
pose(:,17) = [-0.29012934960595865, -0.5869271440134958, -0.10970603662449735, -2.1208745668762075, 0.6791553153892138, 2.2173806979731383]';
pose(:,18) = [-0.308262433956331, -0.6236100905143392, -0.11656266391352842, -2.25342922730597, 0.7216025226010396, 2.355966991596459]';
pose(:,19) = [-0.32639551830670344, -0.6602930370151827, -0.12341929120255951, -2.3859838877357333, 0.7640497298128655, 2.4945532852197805]';
pose(:,20) = [-0.34452860265707586, -0.6969759835160262, -0.1302759184915906, -2.518538548165496, 0.8064969370246913, 2.6331395788431013]';
pose(:,21) = [-0.3626616870074483, -0.7336589300168697, -0.13713254578062167, -2.651093208595259, 0.8489441442365172, 2.7717258724664227]';

setting_time = (length(pose)-1)*Ts;
setpoint = zeros(14, length(pose));

for itter_pose = 1:length(pose)
   setpoint(1:8,itter_pose) = vec8(denso.fkm(pose(:,itter_pose)));
end

for k = 1:nlobj.PredictionHorizon*2
    setpoint(:, length(pose) + k) = setpoint(:, length(pose));
end

%% Closed loop control law

x = x0;
data = [vec8(xm)', theta'];

dtheta = zeros(6, 1);

k = 1;

mv = zeros(1,6);

options = nlmpcmoveopt;

tic
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
timeElapsed = toc
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
% ylim([-.005, .005]);
hold off

subplot(4, 2, 3);
plot(t, data(:, 3));
hold on
plot(t, setpoint(3, :));
xlabel("t(s)");
legend('x_{m_3}', 'x_{d_3}');
xlim([0, t(end)]);
% ylim([-2, 2]);
hold off

subplot(4, 2, 4);
plot(t, data(:, 4));
hold on
plot(t, setpoint(4, :));
xlabel("t(s)");
legend('x_{m_4}', 'x_{d_4}');
xlim([0, t(end)]);
% ylim([-.005, .005]);
hold off

subplot(4, 2, 5);
plot(t, data(:, 5));
hold on
plot(t, setpoint(5, :));
xlabel("t(s)");
legend('x_{m_5}', 'x_{d_5}');
xlim([0, t(end)]);
% ylim([-.005, .005]);
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
% ylim([-.005, .005]);
hold off

subplot(4, 2, 8);
plot(t, data(:, 8));
hold on
plot(t, setpoint(8, :));
xlabel("t(s)");
legend('x_{m_8}', 'x_{d_8}');
xlim([0, t(end)]);
hold off