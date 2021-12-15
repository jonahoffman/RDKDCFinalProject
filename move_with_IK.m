function error = move_with_IK(current, final, ur5)
    % Set step size based on larger of degrees or distance:
    mani_limit = 0.005;
    rot = final(1:3, 1:3) - current(1:3, 1:3); 
    angles = EULERXYZINV(rot); 
    largest_angle = rad2deg(max(angles));  % in deg
    trans = final(1:3, 4) - current(1:3, 4); 
    dist = norm(trans)*100; % in cm
    n = ceil(max(dist, largest_angle) * 1.25); % assume 1 cm or 1 deg per time step with 1.25 safety buffer
    t_step = 10;
    q_current = ur5.get_current_joints();
    for i = 1:n
        trans_interp = trans.*i./n;
        rot_interp = rot.*i./n;
        g_des = create_homog(rot_interp, trans_interp); 
        joint_pos = ur5InvKin(g_des); 
        norms = zeros(6);
        for j = 1:6
            jacobian = ur5BodyJacobian(q_current); 
            mu = manipulability(jacobian, "sigmamin");
            if mu < mani_limit
                norms(j) = inf;
            else
                norms(j) = norm(joint_pos(:, j) - q_current);
            end
        end
        % Find best inverse kin result:
        [~, index] = min(norms);
        ideal_q = joint_pos(:, index);
        
        ur5.move_joints(ideal_q, t_step);
        pause(t_step);
        q_current = ur5.get_current_joints(); 
    end
    current_pose = ur5FwdKin(q_current);
    R = current_pose(1:3, 1:3);
    R_d = final(1:3, 1:3);
    r = current_pose(1:3, 4);
    r_d = final(1:3, 4); 
    dso3 = sqrt(tr((R - R_d)*(R - R_d).'));
    dr3 = norm(r - r_d); 
    error = [dso3 dr3]; 
end