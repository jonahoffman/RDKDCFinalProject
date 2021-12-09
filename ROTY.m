function R = ROTY(theta, ht)

    if ht == false
        R = [cos(theta) 0 sin(theta);  
             0 1 0;
             -sin(theta) 0 cos(theta)];
    else 
        R = [cos(theta) 0 sin(theta) 0;  
             0 1 0 0;
             -sin(theta) 0 cos(theta) 0;
             0 0 0 1];
    end 
end 
    
    
    