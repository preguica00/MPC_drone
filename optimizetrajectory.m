function  [command, optimum, predicted_trajectory] = ...
    optimizetrajectory(current_state, optimum)


    % Initial conditions
    x_init = current_state(1);
    z_init = current_state(2);
    theta_init = current_state(3);
    xvelocity_init = current_state(4);
    zvelocity_init = current_state(5);
    angvelocity_init = current_state(6);
    
    [H, Ts, id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
    
    %initial conditions
    if isempty(optimum)
        optimum = zeros(8*H,1);
        
       % u1_optimum = zeros(H,1);
        %u2_optimum = zeros(H,1);
        %command = [u1_optimum(1), u2_optimum(1)];
    end

   % options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',5000)

    
   [optimum, ~] = fmincon(@(y)costfunction(y, H), optimum);
   %[],[],[],[],[],[],...
    %   @(y)discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,current_state,command));
   
   u1_optimum = optimum(id_u1);
   u2_optimum = optimum(id_u2);
   command = [u1_optimum(1), u2_optimum(1)];


    x_optimum = optimum(id_x);
    z_optimum = optimum(id_z);
    predicted_trajectory = [x_optimum,z_optimum];
end


