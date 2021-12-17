classdef phrase
    %PHRASE An object to represent the collection of letter objects, also
    %provides functions to plan out the overall wirting motion between
    %letters as well as actually executing the motion plan
    %Here for controls, we use RRControl to perfrom the writing
    
    properties
        text; %the inputted text to write
        robot; %ur5_interface object
        points; %the concatenated point path for the given phrase, in the form of a nx3 matrix
        
        % Variables used to determine the largest writing box for a given
        % phrase
        origin; %the bottom left corner coordinates
        ratio; %length ratio of the box containing the phrase written out
        w; %overall width of bounding box after scaling
        h; %height after scaling
        
        %variables defining the workspace of the ur5
        innerR; %the inner radius of the circle where the ur5 can write, determined experimentally
        outerR; %the outer radius o            %disp(err)f the circle where the ur5 can write, determined experimentally
        
        
        spaceW; %the amount of x offset when there is a space
        
        
        liftedHeight; %desired z offset distance from the 'table' when the 'pen' should be lifted
        downHeight; %desired z offset distance from the 'table' when the 'pen' should be down on the paper
        
        maxjointspeed; %max joint speed for homing 
        homeposejoints; %homing joint config
        
        writepose; %the rotational component of the end effector frame wrt to base_frame during writing 
        gripperPose; %the frame transform from tool0 to gripper frame, for calculating z offset
        
        K; %RRControl Gain
    end
    
    methods
        function obj = phrase(inputPhrase,robot, K)
            %When initializing, caller provides the word, ur5 object, and a
            %control gain constant
            
            
            obj.text = inputPhrase;
            obj.robot = robot;
            obj.K = K;
            
            
            %Environmental and ur5 constants
            obj.innerR = 0.4; %Inner and outer radii of workspace determined experimentally
            obj.outerR = 0.6;
            obj.liftedHeight = 0.1; %arbitrary z height for when the 'pen' is lifted
            obj.downHeight = 0.05; %same for when it is down. Recommend avoiding 0 since that will trigger RRcontrol's floor avoidance logic
            obj.maxjointspeed = 0.3; %(rad/s), only used for homing without RRcontrol
            obj.writepose = quat2rotm([0 1 0 0]); %pose of the end effector during writing
            obj.homeposejoints = [1.2900 -1.1000 1.4200 -1.9478 -1.5080 -0.1257]'; %default home pose
            obj.gripperPose = [-0.0000   -1.0000    0.0000   -0.0002;...
                                1.0000   -0.0000    0.0099    0.0013;...
                               -0.0099    0.0000    1.0000    0.1303;...
                                0         0         0         1.0000];
            
            %finding the proper x offset for spaces
            curLetter = letter('o',1);
            obj.spaceW = curLetter.maxWidth * 0.4;

            %call getOrigin function to populate the remaining fields (including scaled points, path, origin coordinates, etc.)
            obj = obj.getOrigin();
        end
        
        %fucntion for getting the overall width of a letter or any
        %path array
        function w = getMaxW(obj,arr)
            A = arr;
            A(A(:, 3)== -1, :)= [];
            A(A(:, 3)== -2, :)= [];
            minX = min(A(:,1));
            maxX = max(A(:,1));
            w = maxX - minX;
        end
        
        %same but for height
        function h = getMaxH(obj,arr)
            A = arr;
            A(A(:, 3)== -1, :)= [];
            A(A(:, 3)== -2, :)= [];
            minH = min(A(:,2));
            maxH = max(A(:,2));
            h = maxH - minH;
        end
        
        %scale a set of points by a constant, without changing the point
        %'mode' in column 3
        function A = scale(obj, arr, c)
            A = arr;
            for i = 1:size(arr,1)
                if arr(i,3) ~= -1
                    A(i,1) = arr(i,1)*c;
                    A(i,2) = arr(i,2)*c;
                end
            end
        end
        
        %find the length ratio of the input letters
        function obj = findRatio(o)
            obj = o;
            
            points = obj.write(1);
            w = obj.getMaxW(points);
            h = 1;
            
            obj.ratio = w/h;
            obj.points = points;
            
        end
        
        %get the origin of the scaled bounding box, this will be the bottom
        %left corner of the box
        function obj = getOrigin(o)
            obj = o;
            obj = obj.findRatio();
            obj = obj.findMaxBox();
        end
        
        %given a width/height ratio, determine the origin coordinates and
        %adjust the point paths to this new 2D frame
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
        
        
        %given a phrase the write, build the point path with a default
        %height. This will be scaled and translated later.
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
        
        % Homing function for the robot, even when it is initialized in a
        % singularity position by directly using move_joints. 
        % Recommended that this function be called before attempting
        % anything involving RRcontrol
        function homenoRR(obj)

            %get current joint config, use the maxjointspeed to determine
            %what the duration of the movement should be, then call
            %move_joints
            qcur = obj.robot.get_current_joints();
            maxqerr = max(abs(obj.homeposejoints - qcur));
            Tstep = maxqerr / obj.maxjointspeed;
            obj.robot.move_joints(obj.homeposejoints, Tstep);
            pause(Tstep);
            
        end
        
        % High-level function for actually drawing the phrase.
        % there are 2 modes, one draws the points in context of a 2D
        % representation of the ur5's workspace in matlab ('mplot' arg),
        % the other draws it in rviz
        function draw(obj, mplot)
            
            %draw phrase in matlab 2D plot
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
                
            %draw the phrase in 3D (including lifting pen/putting down pen for stroke ending) in rviz
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
                            disp('drawing point ' + string(i) + ' out of ' + string(size(obj.points,1)-1));
                            obj.moveto(x, y, 0, i);
                        end
                        
                    end
                end
                
                disp('writing complete!')
                
                %lift the robot directly up after the last point
                obj.liftStraightUp(0);
            
            end
        end
        
        %helper for lifting the robot directly up in +z
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
        
        %helper for lifting the robot directly down in -z
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
        
        %Mid level controlling function to move the robot to a 2D
        %coordinate in either lifted or down position. If marker is
        %defined, it will instruct rviz to visibly mark the point in the
        %ismulation. 
        function moveto(obj, x, y, lifted, marker)
            
            ztarget = obj.downHeight + obj.gripperPose(3,4);
            
            if lifted
                ztarget = obj.liftedHeight + obj.gripperPose(3,4);
            end
            
            g = [obj.writepose [x y ztarget]'; [0 0 0 1]];
            

            err = ur5RRcontrol(g, obj.K, obj.robot,0);
            
            if marker ~= 0
                framename = string(marker);
                Frame = tf_frame('base_link',framename,g*obj.gripperPose);
            end
            
        end
    end
end