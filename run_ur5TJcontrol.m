function finalerr = run_ur5TJcontrol(g_init, g_final, K, ur5)
    % Create hovering frames
    hover = 0.5; 
    hover_init = g_init;
    hover_init(3,4) = hover_init(3,4) + hover;
    hover_final = g_final;
    hover_final(3,4) = hover_final(3,4) + hover; 
    
    finalerr = zeros(7,2);
    finalerr(1,:) = ur5TJcontrol(hover_init, K, ur5);
    % Robot moves straight down to the start location.
    finalerr(2,:) = ur5TJcontrol(g_init, K, ur5); 
    % Robot moves back to initial location (above inital).
    finalerr(3,:) = ur5TJcontrol(hover_init, K, ur5);
    % Robot moves above the target location. 
    finalerr(4,:) = ur5TJcontrol(home, K, ur5);
    finalerr(5,:) = ur5TJcontrol(hover_final, K, ur5);
    % Robot moves down to end location.
    finalerr(6,:) = ur5TJcontrol(g_final, K, ur5);
    % Robot moves back to above the target location.
    finalerr(7,:) = ur5TJcontrol(hover_final, K, ur5); 
end
