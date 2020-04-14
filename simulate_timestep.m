function current_state = simulate_timestep(current_state, command)

    [H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
    tspan = [0 Ts];

%    tSpan = [0, 10];
%y0 = [0; 10; 0; 0; 0; 0; 10; 10];
%ySol = ode45(@quadcopterController, tSpan, y0);
    my_ode = @(t,y) quadcopter_ode(t,y, command);
    [~, y] = ode45(my_ode,tspan, current_state);
    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6)];
end