classdef stroke
    %STROKE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points;
    end
    
    methods
        function obj = stroke(points)
            %STROKE Construct an instance of this class
            %   Detailed explanation goes here
            obj.points = points;
        end
        
        function output = getPoints(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            output = [obj.points ones(length(obj.points), 1); [-1 -1 -1]];
        end
        
        function w = getWidth(obj)
            minX = min(obj.points(:,1));
            maxX = max(obj.points(:,1));
            w = maxX - minX;
        end
        
        function h = getHeight(obj)
            minY = min(obj.points(:,2));
            maxY = max(obj.points(:,2));
            h = maxY - minY;
        end
    end
end

