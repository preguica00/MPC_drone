function [c, ceq] = discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,u1_init,u2_init)

[H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
[xobs,yobs,obj_coord, radius] = obstacle;

% Unpacking
x = y(id_x);
z = y(id_z);
theta = y(id_theta);
x_velocity = y(id_dotx);
z_velocity = y(id_dotz);
angular_velocity = y(id_dottheta);
mode_diff = y(id_u1);
mode_common = y(id_u2);


x = [x_init; y(id_x)];
z = [z_init; y(id_z)];
theta = [theta_init; y(id_theta)];
x_velocity = [xvelocity_init; y(id_dotx)];
z_velocity = [zvelocity_init; y(id_dotz)];
angular_velocity = [angvelocity_init; y(id_dottheta)];
mode_diff = [u1_init; y(id_u1)];
mode_common = [u2_init; y(id_u2)];

% Run discrete prediction
ceq = [];
for i = 1:H
    ceq(end+1) = x(i+1) - (x(i) + Ts*x_velocity(i));
    ceq(end+1) = z(i+1) - (z(i) + Ts*z_velocity(i));
    ceq(end+1) = theta(i+1) - (theta(i) + Ts*angular_velocity(i));
    
    ceq(end+1) = x_velocity(i+1) - (x_velocity(i) + (Ts/mass)*sin(theta(i))*mode_common(i));
    ceq(end+1) = z_velocity(i+1) -(z_velocity(i)+ Ts*(gravitational_acceleration-((1/mass)*cos(theta(i))*mode_common(i))));
    ceq(end+1) = angular_velocity(i+1) - (angular_velocity(i) + Ts*(arm_moment/inertia_moment)*mode_diff(i));
%     ceq(end+1) = y(i+1) - (simulate_timestep(current_state, [u1(k);u2(k)]);
% for i = 1:H
%     ceq(end+1) = x1(i+1) - (x1(i) + Ts*v(i)*cos(theta(i)));
%     ceq(end+1) = x2(i+1) - (x2(i) + Ts*v(i)*sin(theta(i)));
% end

end
% x = y(id_x);
% z = y(id_z);
% c = sum(radius) - vecnorm([x';z']-obj_coord);
c=[];
end