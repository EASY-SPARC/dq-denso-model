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