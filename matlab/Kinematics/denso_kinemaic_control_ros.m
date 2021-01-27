clear all;
close all;
clear classes;
clc;

%% Start and Connect ROS

setenv('ROS_MASTER_URI','http://172.16.66.22:11311')
setenv('ROS_IP','172.16.66.22')
rosinit;

%%
%Create a new DQ_kinematics object with the Denso standard Denavit-Hartenberg parameters           
denso_kine = DQ_DENSO;

% Basic definitions for the simulation
theta = [0,0,0,0,0,0]';     

% Move Arm

position = [0.3892, 0.154 ,0.1981]


thetad = [0,0,0,0,0,0]';

phi = pi/2;

n_y =1;
n_x =0;
n_z =0;

n_vec = [n_x,n_y,n_z];
n = n_vec/norm(n_vec);

n = DQ([0, n_vec(1), n_vec(2), n_vec(3)]);

theta1 = atan2(position(2), position(1));
rz = cos(theta1/2) + sin(theta1/2)*DQ([0, 0, 0, 1]);

ry = cos(pi/4) + sin(pi/4)*DQ([0, 0, 1, 0]);

r = cos(phi/2) + sin(phi/2)*n;

r = ry * r;

p = DQ([0, position(1), position(2), position(3)]);

xd = r + 1/2 * DQ.E * p * r;

epsilon = 0.001;
K = 0.5;
error = epsilon+1;
lambda = 0.5;

% Create Publisher of joints_states

[pub, msg] = rospublisher('/ik_joint_states', 'sensor_msgs/JointState');
msg.Name = [{'joint1'},{'joint2'},{'joint3'},{'joint4'},{'joint5'},{'joint6'}, {'gripper_finger1_joint'}];


pause(1);

% Create Subscriber


while norm(error) > epsilon 
        
    jacob = denso_kine.jacobian(theta);
    xm = denso_kine.fkm(theta);
    error = vec8(xd-xm);
    
    jacob_pinv = (transpose(jacob)/(jacob*transpose(jacob) + (lambda^2)*eye(8)));
        
     
    theta = theta + K*jacob_pinv*error;
    
    theta1 = theta(1,:);
    theta2 = theta(2,:);
    theta3 = theta(3,:);
    theta4 = theta(4,:);
    theta5 = theta(5,:);
    theta6 = theta(6,:);
    
    msg.Position = [theta1,theta2,theta3,theta4,theta5,theta6, 1.0];
    send(pub,msg);
    
    pause(0.1)
    display('Execute')
    
    sub = rossubscriber('/posestamped');
    my_sub= receive(sub,10);
    
    new_position = [my_sub.Pose.Position.X,my_sub.Pose.Position.Y,my_sub.Pose.Position.Z];
    
    if (new_position ~= position)
        xd = set_xd(new_position)
    end
    
end

