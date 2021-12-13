function circle = dot(centerX, centerY, r)

    n = 3;

    theta = 0:2*pi/n:2*pi;
    circle = nan(numel(theta), 2);
    for row = 1:numel(theta)
        circle(row, 1) = cos(theta(row))*r;
        circle(row, 2) = sin(theta(row))*r;
    end

    circle(:,1) = circle(:,1) + centerX;
    circle(:,2) = circle(:,2) + centerY;

end

