function error = ur5IKcontrol(gdesired, ur5)
    % Set step size based on larger of degrees or distance:
    mani_limit = 0.001;
    n = 100; 
    frame = tf_frame('base_link', 'desired_pose', gdesired); 
    q_current = ur5.get_current_joints();
    q_target = ur5InvKin(gdesired);
    norms = zeros(8);
    for j = 1:8
        jacobian = ur5BodyJacobian(q_target(:, j)); 
        mu = manipulability(jacobian, "sigmamin");
        if mu < mani_limit
            norms(j) = inf;
        else
            norms(j) = norm(q_target(:, j) - q_current);
        end
    end
    % Find best inverse kin result:
    [~, index] = min(norms);
    i = index(1); 
    ideal_q = q_target(:, i);
    q_diff = ideal_q - q_current;
    last_q = q_current;
    for k = 1:n
        updated_q = q_current + q_diff.*k./n;
        g_st = ur5FwdKin(updated_q);
        if g_st(3,4) < 0
            disp("Floor contact warning."); 
            q_new = updated_q + [0 pi/6 0 0 0 0]';
            ur5.move_joints(q_new, 10);
            g_new = ur5FwdKin(q_new);
            err = ur5IKcontrol(g_new, ur5);
        end
        jacob = ur5BodyJacobian(updated_q); 
        mu = manipulability(jacob, "sigmamin");
        if mu < mani_limit
            disp("Singularity found - aborting.");
            error = -1;
            return
        end
        mvmt_t = max(abs(updated_q - last_q)/(ur5.speed_limit*pi)*60);
        ur5.move_joints(updated_q, mvmt_t);
        pause(mvmt_t);
        last_q = ur5.get_current_joints();
    end
    ur5.move_joints(ideal_q, 1);
    q_final = ur5.get_current_joints();
    current_pose = ur5FwdKin(q_final);
    R = current_pose(1:3, 1:3);
    R_d = gdesired(1:3, 1:3);
    r = current_pose(1:3, 4);
    r_d = gdesired(1:3, 4); 
    dso3 = sqrt(trace((R - R_d)*(R - R_d).'));
    dr3 = norm(r - r_d); 
    error = [dso3 dr3]; 
end