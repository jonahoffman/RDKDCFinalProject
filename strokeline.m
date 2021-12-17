function output = strokeline(ax, ay, bx, by)
%LINE Summary of this function goes here
%   Detailed explanation goes here

    n = 10;
    
    if ax > bx
        x = bx:(abs(ax-bx))/n:ax;
        x = flip(x);
    elseif ax < bx
        x = ax:(abs(ax-bx))/n:bx;
    else
        x = zeros(1,n+1)+ax;
    end
    
    if ay > by
        y = by:(abs(ay-by))/n:ay;
        y = flip(y);
    elseif ay < by
        y = ay:(abs(ay-by))/n:by;
    else
        y = zeros(1,n+1)+ay;
    end
    
    %disp(x')
    %disp(y')
    
    output = [x' y'];
    

end

