function dxdt = quadcopter_ode(y,u)

[H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;

%% Unpack the state and input vectors

    position_x= y(1);
    position_z= y(2);
    pitch= y(3);
    velocity_x= y(4);
    velocity_z= y(5);
    velocity_pitch= y(6);

    diff_mode  = u(1);
    common_mode = u(2);


%%Equations of motion
x_acceleration = (1/mass)*sin(pitch)* common_mode;
z_acceleration = gravitational_acceleration -(1/mass)*cos(pitch)* common_mode;
pitch_acceleration = (arm_moment/inertia_moment)*diff_mode;

%% 
dxdt(1)= x_acceleration;
dxdt(2)= z_acceleration;
dxdt(3)= pitch_acceleration;
    
end