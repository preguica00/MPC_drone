function [c, ceq] = discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,current_state,command)

[H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
 current_state = simulate_timestep(current_state, command);
%[xobs,yobs,obj_coord, radius] = obstacle;

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


% Run discrete prediction
ceq = [];
for i = 1:H
   % ceq(end+1) = x2(i+1) - (x2(i) + Ts*v(i)*sin(theta(i)));
   w=[x(i+1),z(i+1),theta(i+1),x_velocity(i+1),z_velocity(i+1),angular_velocity(i+1)];
    ceq = [ceq; w - simulate_timestep(current_state, command)];

    end


% x1 = y(id_x1);
% x2 = y(id_x2);
% c = sum(radius) - vecnorm([x1';x2']-obj_coord);
        
end