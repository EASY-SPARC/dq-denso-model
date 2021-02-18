theta0 = [0, 0, -pi/2, 0, 0, 0]';
theta = theta0;
thetad_dotH = [];

for k = 2:length(setpoint(1,:))
    x_dot = (setpoint(1:8,k) - setpoint(1:8,k-1))/Ts
    thetad_dot = pinv(denso.jacobian(theta0))*x_dot
    
    theta_n = theta(:,end)+thetad_dot*Ts;
    
    theta = [theta theta_n];
    thetad_dotH = [thetad_dotH thetad_dot];
end



% theta0 = [0, 0, -pi/2, 0, 0, 0]';
% theta = theta0;
% theta_n = theta;
% thetad_dotH = [];
% 
% setpoint = [x setpoint];
% 
% while (k < (setting_time/Ts)+1)
%     
%     x_dot = (setpoint(1:8,k+1) - setpoint(1:8,k))/Ts
%     thetad_dot = pinv(denso.jacobian(theta_n))*x_dot
%     
%     theta_n = theta(:,end)+thetad_dot*Ts;
%     
%     theta = [theta theta_n];
%     thetad_dotH = [thetad_dotH thetad_dot];
%     
%     setpoint(1:8,k+1)
%     
% %     x = x + densoStateFunction(x, thetad_dot)*Ts
%     x_n = [vec8(denso.fkm(theta_n)); theta_n];
%     
%     ((x_n(1:8) - x(1:8))/Ts)  - x_dot
%     
%     plot(denso, theta_n');
%     drawnow;
%     
%     if (k<size(setpoint,2))
%         k = k+1;
%     end
%     
%     data = [data; x_n'];
%     
%     x = x_n;
% end