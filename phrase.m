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
        maxjointspeed;
        writepose;
        homepose;
    end
    
    methods
        function obj = phrase(inputPhrase,robot)
            obj.text = inputPhrase;
            obj.robot = robot;
            
            
            obj.innerR = 0.4;
            obj.outerR = 0.6;
            obj.defaultH = 1;
            obj.liftedHeight = 0.08;
            obj.downHeight = 0.00;
            obj.endeffectorspeed = 0.2; %(units/sec)
            obj.maxjointspeed = 0.3; %(rad/s)
            obj.writepose = quat2rotm([0 1 0 0]);
            obj.homepose = [1.2673 -0.9105 1.9728 -2.6330 -1.5708 -0.3034]';
            
            
            
            
            curLetter = letter('o',obj.defaultH);
            obj.spaceW = curLetter.maxWidth * 0.4;
            obj = obj.getOrigin();
            
            %obj.draw(sim);
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
            obj.points(:,1) = obj.points(:,1) + obj.origin(1);
            obj.points(:,2) = obj.points(:,2) + obj.origin(2);
        end
        
        function outputPoints = write(obj, h)
            
            xpos = 0;
            delim = [-1 -1 -1; -1 -1 -2];
            outputPoints = delim;
            
            for i = 1:numel(obj.text)
                curLetter = letter(obj.text(i),h);
                cur = curLetter.points;
                cur(:,1) = cur(:,1) + xpos;
                xpos = xpos+curLetter.maxWidth+obj.spaceW;
                outputPoints = [outputPoints; delim; cur];
            end
        end
        
        function home(obj)
            %get current joint config
            qcur = obj.robot.get_current_joints();
            maxqerr = max(abs(obj.homepose - qcur));
            Tstep = maxqerr / obj.maxjointspeed;
            obj.robot.move_joints(obj.homepose, Tstep);
            pause(Tstep);
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
                        scatter(obj.points(i,1),obj.points(i,2))
                    end
                end
                hold off
                
            elseif strcmp(mplot, 'rviz')
                
                
                
                x = obj.points(5,1);
                y = obj.points(5,2);    
                        
                obj.moveto(x, y, 1, 0);
                obj.moveDown(0);
                
                lifted = 0;
                
                for i = 5:(size(obj.points,1)-1)
                    
                    disp(obj.points(i,:));
                                        
                    if obj.points(i,3) == -1
                        disp('end of stroke');
                    end
                    
                    if obj.points(i, 3) == -2
                        disp('lifting pen');
                        obj.liftStraightUp(0);
                        lifted = 1;
                    end
                    
                    if obj.points(i,3) == 1
                        
                        x = obj.points(i,1);
                        y = obj.points(i,2);
                        
                        if lifted == 1
                            obj.moveto(x, y, 1, 0);
                            obj.moveDown(i);
                            lifted = 0;
                        else
                            obj.moveto(x, y, 0, i);
                        end
                        
                    end
                end
                
                obj.liftStraightUp(0);
                obj.home()
            
            end
        end
        
        function liftStraightUp(obj, marker)
            
            %get current position
            %move to lifted position based on const parameter
            %use obj.moveto
            
            qcur = obj.robot.get_current_joints();
            gcur = ur5FwdKin(qcur);
             
            xcur = gcur(1,4);
            ycur = gcur(2,4);
            
            obj.moveto(xcur,ycur, 1, marker);
            
        end
        
        function moveDown(obj, marker)
            
            %get current position
            %move to down position based on const parameter
            %use obj.moveto
            qcur = obj.robot.get_current_joints();
            gcur = ur5FwdKin(qcur);
             
            xcur = gcur(1,4);
            ycur = gcur(2,4);
            
            obj.moveto(xcur,ycur, 0, marker);
            
            
        end
        
        function moveto(obj, x, y, lifted, marker)
            
            ztarget = obj.downHeight;
            
            if lifted
                ztarget = obj.liftedHeight;
            end
            
            
            %get current position (xcur, ycur, zcur)
            qcur = obj.robot.get_current_joints();
            gcur = ur5FwdKin(qcur);
            
            xcur = gcur(1,4);
            ycur = gcur(2,4);
            zcur = gcur(3,4);

            dist = norm([x-xcur y-ycur ztarget-zcur]);
            Tstep = dist/obj.endeffectorspeed;
            %disp(dist);
            %disp(Tstep);
            
            g = [obj.writepose [x y ztarget]'; [0 0 0 1]];
            
            %disp(g);
            
            %move to (g)
            
            qpos = ur5InvKin(g);
            qdiff = 1000;
            index = 0;
            for i = 1:size(qpos,2)
                a = norm(qpos(:,i) - qcur);
                if qdiff > a
                    qdiff = a;
                    index = i;
                end
            end
            
            %disp([qcur';qpos(:,index)']);
            
            maxqerr = max(abs(qpos(:,index) - qcur));
            if obj.maxjointspeed < (maxqerr / Tstep)
                %disp('using joint max speed')
                Tstep = maxqerr / obj.maxjointspeed;
            end
            
            if marker ~= 0
            
                framename = string(marker);
                Frame = tf_frame('base_link',framename,g);
                
            end
            
            
            obj.robot.move_joints(qpos(:,index), Tstep);
            pause(Tstep);

%             g = [obj.writepose [x y ztarget]'; [0 0 0 1]];
%             
%             disp(g);
             
%             
%             ur5RRcontrol(g, obj.K, obj.robot);
            
%            if lifted
%                obj.moveDown()
%            end
            
        end
    end
end

