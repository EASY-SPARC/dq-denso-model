syms theta1 theta2 theta3 theta4 theta5 theta6

thetas = [theta1 theta2 theta3 theta4 theta5 theta6];

denso_DH_theta=  [0,pi/2,-pi/2,0,0,0];
denso_DH_d =     [0.125,0,0,0.210,0,0.070];
denso_DH_a =     [0,0.210, -0.075, 0, 0,0];
denso_DH_alpha = [pi/2,0,-pi/2, pi/2,-pi/2,0];

r_z_thetas = [];
r_x_alphas = [];
x = [];

for i = 1:6
    r_z_thetas = [r_z_thetas rotation_z(thetas(i) + denso_DH_theta(i))];
    r_x_alphas = [r_x_alphas rotation_x(denso_DH_alpha(i))];
    x = [x hemi(r_z_thetas(:,i))*r_x_alphas(:,i)];
end

% r_z_theta1 = rotation_z(thetas(1));
% r_z_theta2 = rotation_z(thetas(2) + pi/2);
% r_z_theta3 = rotation_z(thetas(3) - pi/2);
% r_z_theta4 = rotation_z(thetas(4));
% r_z_theta5 = rotation_z(thetas(5));
% r_z_theta6 = rotation_z(thetas(6));
% 
% r_x_alpha1 = rotation_x(pi/2);
% r_x_alpha2 = rotation_x(0);
% r_x_alpha3 = rotation_x(-pi/2);
% r_x_alpha4 = rotation_x(pi/2);
% r_x_alpha5 = rotation_x(-pi/2);
% r_x_alpha6 = rotation_x(0);
% 
% x0 = hemi(r_z_theta1)*r_x_alpha1;
% x1 = hemi(r_z_theta2)*r_x_alpha2;
% x2 = hemi(r_z_theta3)*r_x_alpha3;
% x3 = hemi(r_z_theta4)*r_x_alpha4;
% x4 = hemi(r_z_theta5)*r_x_alpha5;
% x5 = hemi(r_z_theta6)*r_x_alpha6;

x_0_i = x(:,1);

x_0_i = [x_0_i [(2^(1/2)*cos(theta1/2 + theta2/2 + pi/4))/2;
                (2^(1/2)*cos(theta1/2 - theta2/2 - pi/4))/2;
                (2^(1/2)*sin(theta1/2 - theta2/2 - pi/4))/2;
                (2^(1/2)*sin(theta1/2 + theta2/2 + pi/4))/2]];

x_0_i = [x_0_i [(cos(-theta1/2 + theta2/2 + theta3/2) + cos(theta1/2 + theta2/2 + theta3/2))/2;
                (cos(-theta1/2 + theta2/2 + theta3/2) - cos(theta1/2 + theta2/2 + theta3/2))/2;
               -(sin(-theta1/2 + theta2/2 + theta3/2) + sin(theta1/2 + theta2/2 + theta3/2))/2;
               (-sin(-theta1/2 + theta2/2 + theta3/2) + sin(theta1/2 + theta2/2 + theta3/2))/2]];

x_0_i = [x_0_i [(2^(1/2)*(cos(theta1/2 + theta2/2 + theta3/2)*cos(theta4/2) + sin(-theta1/2 + theta2/2 + theta3/2)*sin(theta4/2)))/2;
                (2^(1/2)*(cos(-theta1/2 + theta2/2 + theta3/2)*cos(theta4/2) - sin(theta1/2 + theta2/2 + theta3/2)*sin(theta4/2)))/2;
                (2^(1/2)*(cos(theta1/2 + theta2/2 + theta3/2)*sin(theta4/2) - sin(-theta1/2 + theta2/2 + theta3/2)*cos(theta4/2)))/2;
                (2^(1/2)*(cos(-theta1/2 + theta2/2 + theta3/2)*sin(theta4/2) + sin(theta1/2 + theta2/2 + theta3/2)*cos(theta4/2)))/2]];

x_0_i = [x_0_i [(cos(theta1/2+theta2/2+theta3/2)*cos(theta5/2-theta4/2) + cos(-theta1/2+theta2/2+theta3/2)*cos(theta5/2+theta4/2) - sin(theta1/2+theta2/2+theta3/2)*sin(theta5/2+theta4/2) - sin(-theta1/2+theta2/2+theta3/2)*sin(theta5/2-theta4/2))/2;
                ()/2;
                ()/2;
                ()/2]];

for i = 5:6
    x_0_i = [x_0_i hemi(x_0_i(:,i-1))*x(:,i)];
end

% x_0_1 = x(:,1);
% x_0_2 = hemi(x_0_1)*x(:,2);
% x_0_3 = hemi(x_0_2)*x(:,3);
% x_0_4 = hemi(x_0_3)*x(:,4);
% x_0_5 = hemi(x_0_4)*x(:,5);
% x_0_6 = hemi(x_0_5)*x(:,6);

z = [];
j = [];
k_hat = [0;0;0;1];

for i = 1:6
    conj_x_0_i = x_0_i(:,i);
    conj_x_0_i(2:4) = -conj_x_0_i(2:4);
    z = [z hemi(hemi(x_0_i(:,i))*k_hat)*conj_x_0_i];
    j = [j hemi(z(:,i))*x_0_i(:,end)];
end





