euler_setpoint = zeros(3,101);
for k = 1:101
    euler_setpoint(:,k) = translation(DQ(setpoint(1:8,k)'));
end