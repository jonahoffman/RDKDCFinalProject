function xi = getXi(g)
    % Obtain a 6x1 unnormalized twist from a 4x4 SE(3) matrix. 
    
    % Extract rotation and translation from transformation matrix:
    rot = g(1:3, 1:3);
    trans = g(1:3, 4);
    % Extract angle from rotation matrix (eqn 2.17)
    angle = acos((trace(rot) - 1)/2);
    
    % Check for angle = 0 condition: 
    if angle == 0
        % Pure translation twist: 
        w = zeros(3,1);
        v = trans;
        xi = [v; w]; 
    else
        % Extract w from rot (eqn 2.18)
        w = (1/(2*sin(angle))).*[g(3,2) - g(2,3); g(1,3) - g(3,1); g(2,1) - g(1,2)];
        % Solve for v using A matrix (proposition 2.9)
        A = (eye(3,3) - rot)*skew3(w) + w*w'.*angle;
        v = inv(A)*trans;
        % Output final twist (multiplied by angle):
        xi = [v; w].*angle; 
    end
end
     