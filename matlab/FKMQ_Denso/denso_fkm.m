%% Load Parameters
syms theta1 theta2 theta3 theta4 theta5 theta6

thetas = [theta1 theta2 theta3 theta4 theta5 theta6];

denso_DH_theta=  [0,pi/2,-pi/2,0,0,0];
denso_DH_d =     [0.125,0,0,0.210,0,0.070];
denso_DH_a =     [0,0.210, -0.075, 0, 0,0];
denso_DH_alpha = [pi/2,0,-pi/2, pi/2,-pi/2,0];

r_z_thetas = [];
r_x_alphas = [];
x = [];

%% Quaternion Rotation Jacobian

for i = 1:6
    r_z_thetas = [r_z_thetas rotation_z(thetas(i) + denso_DH_theta(i))];
    r_x_alphas = [r_x_alphas rotation_x(denso_DH_alpha(i))];
    x = [x hemi(r_z_thetas(:,i))*r_x_alphas(:,i)];
end

x_0_i = x(:,1);

for i = 2:6
    x_0_i = [x_0_i hemi(x_0_i(:,i-1))*x(:,i)];
end

z = [];
J = [];
k_hat = [0;0;0;1];

conj_x_0_i = [1; 0; 0; 0];
conj_x_0_i(2:4) = -conj_x_0_i(2:4);
z = [z hemi(hemi([1; 0; 0; 0])*k_hat)*conj_x_0_i/2];
J = [J hemi(z)*x_0_i(:,end)];

for i = 1:5
    conj_x_0_i = x_0_i(:,i);
    conj_x_0_i(2:4) = -conj_x_0_i(2:4);
    z = [z hemi(hemi(x_0_i(:,i))*k_hat)*conj_x_0_i/2];
    J = [J hemi(z(:,end))*x_0_i(:,end)];
end

J = simplify(J);

%% Geometric Translation Jacobian

T_0_1 = geometric_rotation_z(theta1+denso_DH_theta(1)) * geometric_translation_z(denso_DH_d(1)) * geometric_translation_x(denso_DH_a(1)) * geometric_rotation_x(denso_DH_alpha(1));
T_1_2 = geometric_rotation_z(theta2+denso_DH_theta(2)) * geometric_translation_z(denso_DH_d(2)) * geometric_translation_x(denso_DH_a(2)) * geometric_rotation_x(denso_DH_alpha(2));
T_2_3 = geometric_rotation_z(theta3+denso_DH_theta(3)) * geometric_translation_z(denso_DH_d(3)) * geometric_translation_x(denso_DH_a(3)) * geometric_rotation_x(denso_DH_alpha(3));
T_3_4 = geometric_rotation_z(theta4+denso_DH_theta(4)) * geometric_translation_z(denso_DH_d(4)) * geometric_translation_x(denso_DH_a(4)) * geometric_rotation_x(denso_DH_alpha(4));
T_4_5 = geometric_rotation_z(theta5+denso_DH_theta(5)) * geometric_translation_z(denso_DH_d(5)) * geometric_translation_x(denso_DH_a(5)) * geometric_rotation_x(denso_DH_alpha(5));
T_5_e = geometric_rotation_z(theta6+denso_DH_theta(6)) * geometric_translation_z(denso_DH_d(6)) * geometric_translation_x(denso_DH_a(6)) * geometric_rotation_x(denso_DH_alpha(6));

T_0_2 = T_0_1*T_1_2;
T_0_3 = T_0_2*T_2_3;
T_0_4 = T_0_3*T_3_4;
T_0_5 = T_0_4*T_4_5;
T_0_e = T_0_5*T_5_e;

% T = [];
% for i = 1:6
%     T = [T geometric_rotation_z(thetas(i)+denso_DH_theta(i))*geometric_translation_z(denso_DH_d(i))*geometric_translation_x(denso_DH_a(i))*geometric_rotation_x(denso_DH_alpha(i))];
%     if i > 1
%         T(:,4*i-3:4*i) = T(:,4*(i-1)-3:4*(i-1))*T(:,4*i-3:4*i);
%     end
% end
% 
% T = simplify(T);

z_0 = [0;0;1];
z_1 = T_0_1(1:3, 3);
z_2 = T_0_2(1:3, 3);
z_3 = T_0_3(1:3, 3);
z_4 = T_0_4(1:3, 3);
z_5 = T_0_5(1:3, 3);

p_0 = [0;0;0];
p_1 = T_0_1(1:3, 4);
p_2 = T_0_2(1:3, 4);
p_3 = T_0_3(1:3, 4);
p_4 = T_0_4(1:3, 4);
p_5 = T_0_5(1:3, 4);
p_e = T_0_e(1:3, 4);

% z = [0;0;1];
% p = [0;0;0;1];
% p_e = T(:, end-3:end)*p;
% p_e = p_e(1:3);
% 
% for i = 1:5
%     z = [z T(1:3,4*i-3:4*i-1)*z(:,1)];
%     p = [p T(:,4*i-3:4*i)*p(:,1)];
% end
% p = p(1:3,:);

J_p1 = cross(z_0', (p_e-p_0)')';
J_p2 = cross(z_1', (p_e-p_1)')';
J_p3 = cross(z_2', (p_e-p_2)')';
J_p4 = cross(z_3', (p_e-p_3)')';
J_p5 = cross(z_4', (p_e-p_4)')';
J_p6 = cross(z_5', (p_e-p_5)')';

J = [J_p1 J_p2 J_p3 J_p4 J_p5 J_p6; J];

% J_g = [];
% for i = 1:6
%     J_g = [J_g cross(z(:,i)', (p_e-p(:,i))')'];
% end
% J_g = simplify(J_g);
% digits(5);
% J = [vpa(J_g, 4);J];



