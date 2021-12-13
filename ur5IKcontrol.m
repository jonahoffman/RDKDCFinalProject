function finalerr = ur5IKcontrol(g_init, g_final, ur5)
    
    % Create hovering frames
    hover = 0.5; 
    hover_init = g_init;
    hover_init(3,4) = hover_init(3,4) + hover;
    hover_final = g_final;
    hover_final(3,4) = hover_final(3,4) + hover; 
    
    % Set to home again in case
    
    % In case home position slightly off:
    q = ur5.get_current_joints();
    frame = ur5FwdKin(q); 
    % Number of steps
    % Robot moves so the EE frame goes to pose directly above start location.
    finalerr = zeros(7,2);
    finalerr(1,:) = move_with_IK(frame, hover_init, ur5);
    % Robot moves straight down to the start location.
    finalerr(2,:) = move_with_IK(hover_init, g_init, ur5); 
    % Robot moves back to initial location (above inital).
    finalerr(3,:) = move_with_IK(g_init, hover_init, ur5);
    % Robot moves above the target location. 
    finalerr(4,:) = move_with_IK(hover_init, home, ur5);
    finalerr(5,:) = move_with_IK(home, hover_final, ur5);
    % Robot moves down to end location.
    finalerr(6,:) = move_with_IK(hover_final, g_final, ur5);
    % Robot moves back to above the target location.
    finalerr(7,:) = move_with_IK(g_final, hover_final, ur5); 
end
