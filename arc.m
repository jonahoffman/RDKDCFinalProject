function output = arc(startAng, endAng, r, centerX, centerY, ccw)
%ARC Function for generating a 2D arc as defined by starting angle and
%ending angle, radius of circle, the center of the circle, and whether or
%not the arc is in CCW or CW. The arc will be represented as a sequential order of 2D points. 

    %defult number of points in a circle, which will be indexed to get an arc. Anything above 30 will make the
    %drawing process extremely slow. 
    n = 20;

    %generating a unit circle as a set of points
    theta = 0:2*pi/n:2*pi;
    circle = nan(numel(theta), 2);
    for row = 1:numel(theta)
        circle(row, 1) = cos(theta(row));
        circle(row, 2) = sin(theta(row));
    end
    
    
    %get a subarray of the unit circle as the points for the arc, reversing
    %order if necessary
    startInd = round(startAng/(2*pi)*numel(theta));
    endInd = round(endAng/(2*pi)*numel(theta));
    if startAng == 0
        startInd = 1;
    end
    if endAng == 0
        endInd = 1;
    end
    if endAng == 2*pi
        endInd = numel(theta);
    end
    
    if ccw == 1
        if startInd < endInd
            output = circle(startInd:min([endInd+1 n+1]),:);
        else
            output = [circle(startInd:end,:);circle(1:min([endInd+1 n+1]),:)];
        end
    elseif ccw == 0
        if startInd > endInd
            output = circle(endInd:min([startInd+1 n+1]),:);
            output = flip(output);
        else
            output = [flip(circle(1:startInd,:));flip(circle(endInd-1:end,:))];
        end
    end
    
    %scaling by radius
    output = arrayfun(@(x) r*x, output);

    %offsetting by center coords
    output(:,1) = output(:,1) + centerX;
    output(:,2) = output(:,2) + centerY;

end

