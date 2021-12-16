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
        homeposejoints;
        gripperPose;
        K;
    end
    
    methods
        function obj = phrase(inputPhrase,robot, K)
            obj.text = inputPhrase;
            obj.robot = robot;
            
            
            obj.innerR = 0.4;
            obj.outerR = 0.6;
            obj.defaultH = 1;
            obj.liftedHeight = 0.1;
            obj.downHeight = 0.05;
            obj.endeffectorspeed = 0.2; %(units/sec)
            obj.maxjointspeed = 0.3; %(rad/s)
            obj.K = K;
            obj.writepose = quat2rotm([0 1 0 0]);
            obj.homeposejoints = [1.2900 -1.1000 1.4200 -1.9478 -1.5080 -0.1257]';
            
            obj.gripperPose = [-0.0000   -1.0000    0.0000   -0.0002;...
                                1.0000   -0.0000    0.0099    0.0013;...
                               -0.0099    0.0000    1.0000    0.1303;...
                                0         0         0         1.0000];
            
            
            
            
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
        
        function homenoRR(obj)

            %get current joint config
            qcur = obj.robot.get_current_joints();
            maxqerr = max(abs(obj.homeposejoints - qcur));
            Tstep = maxqerr / obj.maxjointspeed;
            obj.robot.move_joints(obj.homeposejoints, Tstep);
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
                
                disp('moving to default home');
                obj.homenoRR();
                disp('finished homing');
                
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
                            disp('drawing point ' + string(i) + ' out of ' + string(size(obj.points,1)-1));
                            obj.moveto(x, y, 0, i);
                        end
                        
                    end
                end
                
                disp('writing complete!')
                
                obj.liftStraightUp(0);
            
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
            
            ztarget = obj.downHeight + obj.gripperPose(3,4);
            
            if lifted
                ztarget = obj.liftedHeight + obj.gripperPose(3,4);
            end
            
            g = [obj.writepose [x y ztarget]'; [0 0 0 1]];
            

            err = ur5RRcontrol(g, obj.K, obj.robot,0);
            %disp(err)
            
            if marker ~= 0
                framename = string(marker);
                Frame = tf_frame('base_link',framename,g*obj.gripperPose);
            end
            
        end
    end
end