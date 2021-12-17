function output = strokeline(ax, ay, bx, by)
%LINE Function for generating a 2D line as defined by starting xy and
%ending xy. The line will be represented as a series of 2D points. 

    %defult number of points in a line. Anything above 20 will make the
    %drawing process extremely slow. 
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
    
    output = [x' y'];
    

end

