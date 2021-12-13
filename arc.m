function output = arc(startAng, endAng, r, centerX, centerY, ccw)

    n = 30;

    theta = 0:2*pi/n:2*pi;
    circle = nan(numel(theta), 2);
    for row = 1:numel(theta)
        circle(row, 1) = cos(theta(row));
        circle(row, 2) = sin(theta(row));
    end
    
    startInd = round(startAng/(2*pi)*numel(theta));
    endInd = round(endAng/(2*pi)*numel(theta));
    %disp(startInd)
    %disp(endInd)
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
    %disp(output)
    output = arrayfun(@(x) r*x, output);
    
    output(:,1) = output(:,1) + centerX;
    output(:,2) = output(:,2) + centerY;

end

