function R = ROTZ(theta, ht)
   
    if ht == false
        R = [cos(theta) -sin(theta) 0; 
             sin(theta) cos(theta) 0;
             0 0 1]; 
    else 
        R = [cos(theta) -sin(theta) 0 0; 
             sin(theta) cos(theta) 0 0;
             0 0 1 0;
             0 0 0 1];
    end
end 
    
    
    