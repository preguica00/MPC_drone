function cost = costfunction(y, H)

    %final positions
    x_final=60;
    z_final=60;
    theta_final=45;
    dotx_final=0;
    dotz_final=0;
    dottheta_final=0;
    
    [H, Ts, id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;

    % Unpacking
   
    mode_diff = y(id_u1);
    mode_common = y(id_u2);
    x = y(id_x);
    z = y(id_z);
    theta = y(id_theta);
    x_velocity = y(id_dotx);
    z_velocity = y(id_dotz);
    angular_velocity = y(id_dottheta);
   
    
    cost = sum((x(:)-x_final).^2+(z(:)-z_final).^2 + (theta(:)-theta_final).^2 + (x_velocity(:)-dotx_final).^2+ (z_velocity(:)-dotz_final).^2+(angular_velocity(:)-dottheta_final).^2 + (mode_diff(:)).^2+(mode_common(:)).^2);   

end