function finalerr = ur5TJcontrol(gdesired, K, ur5)
% Implements a resolved-rate control system for the UR5 robot. 

    t_step= 0.2; % time step (s)
    vk_min = 1; % (cm)
    wk_min = 1*pi/180; % (rad)
    mani_limit = 0.005; % (unitless)

    while (1<2)
        % Get joint position and calculate fkin / jacobian
        q = ur5.get_current_joints();
        g_st = ur5FwdKin(q);
        Jb = ur5BodyJacobian(q); 
        
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
        q = q - K*t_step*(Jb.'*xi);
        ur5.move_joints(q, t_step*2);
        pause(t_step*2);
        
        % Calculate vk and wk norms and compare to threshold values:
        vk=norm(xi(1:3));
        wk=norm(xi(4:6));
        if ((vk < vk_min) && (wk < wk_min))
            finalerr = vk * 100; 
            disp("Done movement - all clear.");
            return
        end

        pause(t_step); 
    end
    q_current = ur5.get_current_joints();
    current_pose = ur5FwdKin(q_current);
    R = current_pose(1:3, 1:3);
    R_d = gdesired(1:3, 1:3);
    r = gdesired(1:3, 4);
    r_d = final(1:3, 4); 
    dso3 = sqrt(tr((R - R_d)*(R - R_d).'));
    dr3 = norm(r - r_d); 
    finalerr = [dso3 dr3]; 
end 