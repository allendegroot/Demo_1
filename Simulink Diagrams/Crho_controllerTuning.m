% Rho (dot) = radius * (theta1 (dot) * theta2 (dot))/2
out = sim('Crho_controllerTuning_Diagram');

% Experimental time and velocity defined in workspace


figure(1);
plot(out.actual_linvelocity);
hold on;
plot(experimentalTime, experimentalVelocity);
title('Linear Velocity');
xlabel('Time (seconds)');
ylabel('Linear Velocity (m/s)');

figure(2);
plot(out.Vbar_a);
title('Vbar_A');
xlabel('Time (seconds)');
ylabel('Vbar_A (V)');

figure(3);
plot(out.desired_linvelocity);
title('Desired Linear Velocity');
xlabel('Time (seconds)');
ylabel('Linear Velocity (m/s)');