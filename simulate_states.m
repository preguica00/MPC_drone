load ('states.mat')
load ('control.mat')



%     x = y(id_x);
%     z = y(id_z);
%     theta = y(id_theta);
%     x_velocity = y(id_dotx);
%     z_velocity = y(id_dotz);
%     angular_velocity = y(id_dottheta);
figure
subplot(3,2,1)
plot(state_trajectory(:,1), 'b', 'Linewidth', 1.5);
xlabel('x')
ylabel('Amplitude')

subplot(3,2,2)
plot(state_trajectory(:,2),'b', 'Linewidth', 1.5);
xlabel('z')
ylabel('Amplitude')

subplot(3,2,3)
plot(state_trajectory(:,3),'b', 'Linewidth', 1.5);
xlabel('\theta')
ylabel('Amplitude')

subplot(3,2,4)
plot(state_trajectory(:,4),'b', 'Linewidth', 1.5);
xlabel('$\dot{x}$', 'Interpreter','latex')
ylabel('Amplitude')

subplot(3,2,5)
plot(state_trajectory(:,5),'b', 'Linewidth', 1.5);
xlabel('$\dot{z}$', 'Interpreter','latex')
ylabel('Amplitude')

subplot(3,2,6)
plot(current_MPC_solution(:,6),'b', 'Linewidth', 1.5);
xlabel('$\dot{\theta}$', 'Interpreter','latex')
ylabel('Amplitude')

Figure
subplot(1,2,1)
plot(state_trajectory(:,5),'b', 'Linewidth', 1.5);
xlabel('$\dot{z}$', 'Interpreter','latex')
ylabel('Amplitude')
subplot(1,2,2)
plot(state_trajectory(:,5),'b', 'Linewidth', 1.5);
xlabel('u_2')
ylabel('Amplitude')