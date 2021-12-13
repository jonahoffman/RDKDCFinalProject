classdef phrase
    %PHRASE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        text;
        robot;
        origin;
        ratio;
        points;
        w;
        h;
        innerR;
        outerR;
        spaceW;
        defaultH;
        liftedHeight;
        downHeight;
        endeffectorspeed;
        writepose;
    end
    
    methods
        function obj = phrase(inputPhrase,robot, sim)
            obj.text = inputPhrase;
            obj.robot = robot;
            
            
            obj.innerR = 100;
            obj.outerR = 300;
            obj.defaultH = 1;
            obj.liftedHeight = 40;
            obj.downHeight = 5;
            obj.endeffectorspeed = 20; %(units/sec)
            obj.writepose = [1 0 0; 0 1 0; 0 0 1];
            
            
            
            
            curLetter = letter('o',obj.defaultH);
            obj.spaceW = curLetter.maxWidth * 0.4;
            obj = obj.getOrigin();
            
            obj.draw(sim);
        end
        
        function w = getMaxW(obj,arr)
            A = arr;
            A(A(:, 3)== -1, :)= [];
            A(A(:, 3)== -2, :)= [];
            minX = min(A(:,1));
            maxX = max(A(:,1));
            w = maxX - minX;
        end
        
        function h = getMaxH(obj,arr)
            A = arr;
            A(A(:, 3)== -1, :)= [];
            A(A(:, 3)== -2, :)= [];
            minH = min(A(:,2));
            maxH = max(A(:,2));
            h = maxH - minH;
        end
        
        function A = scale(obj, arr, c)
            A = arr;
            for i = 1:size(arr,1)
                if arr(i,3) ~= -1
                    A(i,1) = arr(i,1)*c;
                    A(i,2) = arr(i,2)*c;
                end
            end
        end
        
        function obj = findRatio(o)
            obj = o;
            
            points = obj.write(obj.defaultH);
            w = obj.getMaxW(points);
            h = obj.defaultH;
            
            obj.ratio = w/h;
            obj.points = points;
            
        end
        
        function obj = getOrigin(o)
            obj = o;
            obj = obj.findRatio();
            obj = obj.findMaxBox();
        end
        
        function obj = findMaxBox(o)
            obj = o;
            m = obj.ratio/2;
            a = m^2 + 1;
            b = 2*obj.innerR;
            c = obj.innerR^2 - obj.outerR^2;

            r = roots([a b c]);

            y = 0;

            if r(1) > 0
                y = r(1);
            else
                y = r(2);
            end

            x = m*y;

            obj.origin = [-x obj.innerR];
            obj.w = x;
            obj.h = y;
            
            obj.points = obj.scale(obj.points, y);
        end
        
        function outputPoints = write(obj, h)
            
            xpos = 0;
            outputPoints = [-1 -1 -1; -1 -1 -2];
            
            for i = 1:numel(obj.text)
                curLetter = letter(obj.text(i),h);
                cur = curLetter.points;
                cur(:,1) = cur(:,1) + xpos;
                xpos = xpos+curLetter.maxWidth+obj.spaceW;
                outputPoints = [outputPoints; cur];
            end
        end
        
        function draw(obj, mplot)
            
            if strcmp(mplot, 'mplot')
            
                hold on

                th = 0:pi/50:2*pi;
                xunit = obj.innerR * cos(th);
                yunit = obj.innerR * sin(th);
                plot(xunit, yunit);


                xunit22 = obj.outerR * cos(th);
                yunit22 = obj.outerR * sin(th);
                plot(xunit22, yunit22);

                plot([1 1]*obj.w, ylim, '--k')               
                plot([1 1]*-obj.w, ylim, '--k')              

                plot(xlim, [1 1]*obj.innerR, '--k')                
                plot(xlim, [1 1]*(obj.innerR+obj.h), '--k')               

                for i = 1:size(obj.points,1)
                    pause(0.01)
                    if obj.points(i,3) == 1
                        scatter(obj.points(i,1)+obj.origin(1),obj.points(i,2)+obj.origin(2))
                    end
                end
                hold off
                
            elseif strcmp(mplot, 'rviz')
                
                for i = 1:size(obj.points,1)
                    
                    lifted = 0;
                    
                    if obj.points(i,3) == -1
                        i = i + 1;
                        if obj.points(i, 3) == -2
                            obj.liftStraightUp();
                            i = i + 1;
                            lifted = 1;
                        end
                    end
                    
                    if obj.points(i,3) == 1
                        
                        x = obj.points(i,1);
                        y = obj.points(i,2);
                        
                        obj.moveto(x, y, lifted);
                        
                    end
                end
            
            end
        end
        
        function liftStraightUp(obj)
            
            %get current position
            %move to lifted position based on const parameter
            %use obj.moveto
            
        end
        
        function moveDown(obj)
            
            %get current position
            %move to down position based on const parameter
            %use obj.moveto
            
        end
        
        function moveto(obj, x, y, lifted)
            
            ztarget = obj.downHeight;
            
            if lifted
                ztarget = obj.liftedHeight;
            end
            
            %get current position (xcur, ycur, zcur)
            
            dist = norm([x-xcur y-ycur ztarget-zcur]);
            Tstep = dist/obj.endeffectorspeed;
            
            g = [obj.writepose [x y ztarget]'; [0 0 0 1]];
            
            %move to (g)
            
            if lifted
                obj.moveDown()
            end
            
        end
    end
end

