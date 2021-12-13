function [vec, well_defined] = EULERXYZINV(R)
    % Return euler angles from a 3x3 rotation matrix. 
    
    % The function is numerically ill-defined when atan2(y, x) returns an
    % undefined result for any roll/pitch/yaw angle. This occurs when the
    % input to atan2 is of the form: atan2(0, 0). This function outputs a
    % warning and returns the angle = 0.0 in this case. 
    % check argument dimension
    
    [rows, cols] = size(R);
    if ((rows ~= 3) || (cols ~= 3))
      error('EULERXYZINV requires a 3x3 vector argument. Check your dimensions.');
    end
    
    well_defined = true; 
    
    roll_warning = "Undefined pitch condition detected. Returning angle = 0.0. \n";
    y1 = -R(2,3); 
    x1 = R(3,3);
    x = 0.0; 
    
    if (y1 == 0) && (x1 == 0)
        x = 0.0;
        fprintf(roll_warning);
        well_defined = false;
    else 
        x = atan2(y1, x1);
    end
        
    pitch_warning = "Undefined roll condition detected. Returning angle = 0.0. \n";
    y2 = R(1,3); 
    x2 = sqrt((R(2,3)^2) + (R(3,3)^2));
    y = 0.0; 
    
    if (y2 == 0) && (x2 == 0)
        y = 0.0;
        fprintf(pitch_warning);
        well_defined = false;
    else 
        y = atan2(y2, x2);
    end
    
    yaw_warning = "Undefined yaw condition detected. Returning angle = 0.0. \n";
    y3 = -R(1,2);
    x3 = R(1,1); 
    z = 0.0; 
    
    if (y3 == 0) && (x3 == 0)
        z = 0.0;
        fprintf(yaw_warning);
        well_defined = false;
    else 
        z = atan2(y3, x3);
    end
    
    vec = [x; y; z];
end