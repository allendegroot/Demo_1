% Rho (dot) = radius * (theta1 (dot) * theta2 (dot))/2
out = sim('linvelocity_closedLoop');


%Plots the necessary info to ensure correct operation
figure(1);
plot(out.desired_linearPos);
title('Desired Linear Position');
xlabel('Time (seconds)');
ylabel('Linear Position (m)');

figure(2);
plot(out.actual_linearPos);
title('Actual Linear Position');
xlabel('Time (seconds)');
ylabel('Linear Position (m)');

figure(3);
plot(out.desired_linvelocity);
title('Desired Linear Velocity');
xlabel('Time (seconds)');
ylabel('Linear Velocity (m/s)');

figure(3);
plot(out.actual_linvelocity);
title('Actual Linear Velocity');
xlabel('Time (seconds)');
ylabel('Linear Velocity (m/s)');

figure(5);
plot(out.Vbar_a);
title('Vbar_A');
xlabel('Time (seconds)');
ylabel('Vbar_A (V)');