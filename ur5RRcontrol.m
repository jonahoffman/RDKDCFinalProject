function finalerr = ur5RRcontrol(gdesired, K, ur5)
% Implements a resolved-rate control system for the UR5 robot. 

    t_step= 0.5; % time step (s)
    vk_min = 1; % (cm)
    wk_min = 1*pi/180; % (rad)
    mani_limit = 0.005; % (unitless)
    % tool_frame = [ROTX(-pi/2, false)*ROTY(pi/2, false) [0 0 0]'; 0 0 0 1];
    % gtool = gdesired * tool_frame; 
    q_current = ur5.get_current_joints(); 
    frame = tf_frame('base_link', 'desired_pose', gdesired); 
    while (1<2)
        % Get joint position and calculate fkin / jacobian
        g_st = ur5FwdKin(q_current);
        
        if g_st(3,4) < 0
            disp("Floor contact warning."); 
            q_new = q_current + [0 pi/6 0 0 0 0]';
            ur5.move_joints(q_new, 10);
            g_new = ur5FwdKin(q_new);
            err = ur5RRcontrol(g_new, K, ur5);
        end
        
        Jb = ur5BodyJacobian(q_current); 
        
        % Calculate manipulability
        sigma = manipulability(Jb, 'sigmamin');
        
        % Check for singularity and cease operation if found
        if (sigma < mani_limit)
            finalerr = -1;
            disp('Singularity - aborting.'); 
            return
        end

        % Update positions based on RR-control equations: 
        s = gdesired\g_st;
        xi = getXi(s);
        q_next = q_current - K*t_step*(Jb\xi);
        mvmt_t = max(abs(q_next - q_current)/(ur5.speed_limit*pi)*60); 
        ur5.move_joints(q_next, mvmt_t);
        pause(mvmt_t);
        
        % Calculate vk and wk norms and compare to threshold values:
        vk=norm(xi(1:3));
        wk=norm(xi(4:6));
        if ((vk < vk_min) && (wk < wk_min)) 
            disp("Done movement - all clear.");
            break;
        end
        q_current = ur5.get_current_joints(); 
    end
    current_pose = ur5FwdKin(q_current);
    R = current_pose(1:3, 1:3);
    R_d = gdesired(1:3, 1:3);
    r = gdesired(1:3, 4);
    r_d = gdesired(1:3, 4); 
    dso3 = sqrt(trace((R - R_d)*(R - R_d).'));
    dr3 = norm(r - r_d); 
    finalerr = [dso3 dr3]; 
end