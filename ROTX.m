function R = ROTX(theta, ht)

    if ht == false
        R = [1 0 0; 
             0 cos(theta) -sin(theta);
             0 sin(theta) cos(theta)];
    else 
        R = [1 0 0 0; 
             0 cos(theta) -sin(theta) 0;
             0 sin(theta) cos(theta) 0;
             0 0 0 1];
    end
end 
    
    
    