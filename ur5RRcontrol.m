function finalerr = ur5RRcontrol(gdesired, K, ur5, drawDesired)
% Implements a resolved-rate control system for the UR5 robot. 

    t_step= 0.5; % time step (s)
    vk_min = 0.01; % (cm)
    wk_min = 1*pi/180; % (rad)
    mani_limit = 0.001; % (unitless)
    % Set tool to gripper transform:
    tool2gripper = ROTZ(pi/2, true);
    tool2gripper(3,4) = 0.13; 
    % Get current joint config and create desired gripper frame:
    q_current = ur5.get_current_joints();
    
    if drawDesired
        frame = tf_frame('base_link', 'desired_pose', gdesired*tool2gripper); 
    end
    while (1<2)
        % Get joint position::
        g_st = ur5FwdKin(q_current);
        % Implement check to see if robot crosses floor:
        if g_st(3,4) < 0
            disp("Floor contact warning."); 
            q_new = q_current + [0 pi/6 0 0 0 0]';
            ur5.move_joints(q_new, 10);
            g_new = ur5FwdKin(q_new);
            err = ur5RRcontrol(g_new, K, ur5);
        end
        % Implement rough check to avoid joint 3 singularity:
        if abs(q_current(3)) < 0.2
            disp("Avoiding pot. joint 3 sing.");
            new_q = q_current + [0 0 -abs(q_current(3))*2 0 0]';
            ur5.move_joints(new_q, 5);
        elseif abs(q_current(3)) > 2.94
            disp("Avoiding pot. joint 3 sing.");
            new_q = q_current + [0 0 (-(abs(q_current(3))*2 - pi))  0 0]'; 
            ur5.move_joints(new_q, 5);
        end
        % Calculate jacobian and manipulability:
        Jb = ur5BodyJacobian(q_current);
        sigma = manipulability(Jb, 'sigmamin');
        
        % Check for singularity and cease operation if found
        if (sigma < mani_limit)
            finalerr = -1;
            disp('Singularity - aborting.'); 
            return
        end

        % Update positions based on RR-control equations: 
        Jb = ur5BodyJacobian(q_current);
        s = gdesired\g_st;
        xi = getXi(s);
        vk = norm(xi(1:3));
        % Shorten path if long distance, avoid odd motion:
        if vk > 0.5
            xi = xi/4;
        end
        invJ = inv(Jb); 
        q_next = q_current - K*t_step*invJ*xi;
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
    % Get current pose and compute both types of error:
    current_pose = ur5FwdKin(q_current);
    R = current_pose(1:3, 1:3);
    R_d = gdesired(1:3, 1:3);
    r = current_pose(1:3, 4);
    r_d = gdesired(1:3, 4); 
    dso3 = sqrt(trace((R - R_d)*(R - R_d).'));
    dr3 = norm(r - r_d); 
    finalerr = [dso3 dr3]; 
end