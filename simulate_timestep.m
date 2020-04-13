function current_state = simulate_timestep(current_state, command)

    [H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
    tspan = [0 Ts];

    my_ode = @(y,u) quadcopter_ode(y,u);
    [~, y] = ode45(my_ode,tspan, [current_state command]);

    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6)];
    command = [u(1),u(2)];
end