function simulate()

    close all
    clf
    hold on
    plot_prediction = plot(0,0,'or-', 'Linewidth', 1.5);
    plot_trajectory = plot(0,0,'db-','Linewidth', 1.5);
    axis equal
    xlim([0 60])
    ylim([0 60])
    
    current_state = [0; 0;0;0;0;0];
    current_MPC_solution = [];
    
    [H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
    [xobs,yobs, obj_coord,radius] = obstacle;
%     plot(xobs,yobs, '-k','Linewidth', 1.5);

    for k = 1:50
        
        %% Run the controller
        [command, current_MPC_solution, predicted_trajectory] = ...
            optimizetrajectory(current_state, current_MPC_solution);
        
        %% Run the simulation
        current_state = simulate_timestep(current_state, command);

        %% Visualize
        plot_prediction.XData = predicted_trajectory(:,1);
        plot_prediction.YData = predicted_trajectory(:,2);
        plot_trajectory.XData(end+1) = current_state(1);
        plot_trajectory.YData(end+1) = current_state(2);
        
        drawnow
        pause(0.05)
    end

end