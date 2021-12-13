classdef letter
    %LETTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        char;
        height;
        maxWidth;
        points;
        lowerRatio;
    end
    
    methods
        function obj = letter(char,h)
            %LETTER Construct an instance of this class
            %   Detailed explanation goes here
            obj.char = char;
            obj.height = h;
            obj.lowerRatio = (1/1.6);
            obj = obj.writeLetter();
        end
        
        function w = getMaxW(obj,arr)
            A = arr;
            A(A(:, 3)== -1, :)= [];
            minX = min(A(:,1));
            maxX = max(A(:,1));
            w = maxX - minX;
        end
        
        function A = shiftVertical(obj, arr, y)
            A = arr;
            for i = 1:size(arr,1)
                if arr(i,3) ~= -1
                    A(i,2) = arr(i,2) + y;
                end
            end
        end
        
        function A = shiftHorizontal(obj, arr, x)
            A = arr;
            for i = 1:size(arr,1)
                if arr(i,3) ~= -1
                    A(i,1) = arr(i,1) + x;
                end
            end
        end
        
        function A = scaleStroke(obj, arr, x, y)
            A = arr;
            for i = 1:size(arr,1)
                if arr(i,3) ~= -1
                    A(i,1) = arr(i,1)*x;
                    A(i,2) = arr(i,2)*y;
                end
            end
        end
            
        
        function obj = writeLetter(obj)
            
            output = nan(1,3);

            a = obj.height*obj.lowerRatio;
            b = obj.height*(1-obj.lowerRatio) / 2;
            
            dotR = 0.02;
            
            liftPen = [0 0 -2];
            
            %disp(a)
            %disp(b)

            if obj.char == 'a'
                stroke1 = stroke(arc(pi/5, 2*pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w, 0, w, a));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'b'
                stroke1 = stroke(arc(pi+pi/5, pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                stroke2 = stroke(strokeline(0, 0, 0, a+b));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'c'
                stroke1 = stroke(arc(pi/5, 2*pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'd'
                stroke1 = stroke(arc(pi/5, 2*pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w, 0, w, a+b));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'e'
                stroke1 = stroke(arc(2*pi-pi/3,0, a/2, a/2, a/2, 0));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w, a/2, 0, a/2));
                output = [output; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end

            if obj.char == 'f'
                stroke1 = stroke(arc(pi/2,pi, b, 1.5*b, a, 1));
                output = stroke1.getPoints();
                w = stroke1.getWidth()*0.5;
                stroke2 = stroke(strokeline(w, a, w, 0));
                output = [output; stroke2.getPoints()];
                stroke3 = stroke(strokeline(0, a, 3*w, a));
                output = [output; liftPen; stroke3.getPoints()];
                output = obj.shiftVertical(output,b);
            end

            if obj.char == 'g'
                stroke1 = stroke(arc(pi/5, 2*pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w, a, w, 0));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
                
                stroke3 = stroke(arc(2*pi, pi+pi/5, b, 0, 0, 0));
                s3p = obj.scaleStroke(stroke3.getPoints(),w/b/2,1);
                s3p = obj.shiftHorizontal(s3p,w/2);
                s3p = obj.shiftVertical(s3p,b);
                output = [output; s3p];
            end
            
            if obj.char == 'h'
                stroke1 = stroke(strokeline(0, a+2*b, 0, b));
                output = stroke1.getPoints();
                stroke2 = stroke(arc(pi-pi/6, pi/6, a/2, 0, 0, 0));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                s2p = obj.shiftVertical(stroke2.getPoints(),a/2+h);
                s2p = obj.shiftHorizontal(s2p, w/2);
                output = [output; liftPen; s2p];
                stroke3 = stroke(strokeline(w, a-h+b, w, b));
                output = [output; stroke3.getPoints()];
            end
            
            if obj.char == 'i'
                stroke1 = stroke(dot(0, a+b*3/2,dotR));
                output = stroke1.getPoints();
                stroke2 = stroke(strokeline(0, a+b/2, 0, b));
                output = [output; liftPen; stroke2.getPoints()];
            end
            
            if obj.char == 'j'
                stroke1 = stroke(dot(0, a+b*3/2,dotR));
                output = stroke1.getPoints();
                stroke2 = stroke(strokeline(0, a+b/2, 0, b));
                output = [output; liftPen; stroke2.getPoints()];
                stroke3 = stroke(arc(2*pi, pi+pi/5, b, 0, 0, 0));
                w = stroke3.getWidth();
                s3p = obj.scaleStroke(stroke3.getPoints(),w/b/2,1);
                s3p = obj.shiftHorizontal(s3p,w/2);
                s3p = obj.shiftVertical(s3p,b);
                output = obj.shiftHorizontal(output, w);
                output = [output; s3p];
            end
            
            if obj.char == 'k'
                stroke1 = stroke(strokeline(0, a+b+b, 0, b));
                stroke2 = stroke(strokeline(0, b+a/2, a*0.6, a+b));
                stroke3 = stroke(strokeline(0, b+a/2, a*0.6, b));
                
                output = [stroke1.getPoints(); liftPen;stroke2.getPoints(); liftPen;stroke3.getPoints()];
            end
            
            if obj.char == 'l'
                stroke1 = stroke(strokeline(0, 0, 0, a+b));
                output = stroke1.getPoints();
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'm'
                stroke1 = stroke(strokeline(0, a+b, 0, b));
                output = stroke1.getPoints();
                stroke2 = stroke(arc(pi-pi/6, pi/6, a/2, 0, 0, 0));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                s2p = obj.shiftVertical(stroke2.getPoints(),a/2+h);
                s2p = obj.shiftHorizontal(s2p, w/2);
                output = [output; liftPen; s2p];
                stroke3 = stroke(strokeline(w, a-h+b, w, b));
                output = [output; stroke3.getPoints()];
                s4p = obj.shiftHorizontal(s2p, w);
                s5p = obj.shiftHorizontal(stroke3.getPoints(), w);
                output = [output; liftPen; s4p; s5p];
            end
            
            if obj.char == 'n'
                stroke1 = stroke(strokeline(0, a+b, 0, b));
                output = stroke1.getPoints();
                stroke2 = stroke(arc(pi-pi/6, pi/6, a/2, 0, 0, 0));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                s2p = obj.shiftVertical(stroke2.getPoints(),a/2+h);
                s2p = obj.shiftHorizontal(s2p, w/2);
                output = [output; liftPen; s2p];
                stroke3 = stroke(strokeline(w, a-h+b, w, b));
                output = [output; stroke3.getPoints()];
            end
            
            if obj.char == 'o'
                stroke1 = stroke(arc(0, 2*pi, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'p'
                stroke1 = stroke(arc(pi+pi/5, pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                stroke2 = stroke(strokeline(0, a, 0, -b));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'q'
                stroke1 = stroke(arc(pi/5, 2*pi-pi/5, a/2, a/2, a/2, 1));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w, a, w, -b));
                output = [output; liftPen; stroke2.getPoints()];
                output = obj.shiftVertical(output,b);
            end
            
            if obj.char == 'r'
                stroke1 = stroke(strokeline(0, a+b, 0, b));
                output = stroke1.getPoints();
                stroke2 = stroke(arc(pi-pi/6, pi/6, a/2, 0, 0, 0));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                s2p = obj.shiftVertical(stroke2.getPoints(),a/2+h);
                s2p = obj.shiftHorizontal(s2p, w/2);
                output = [output; liftPen; s2p];
            end
            
            if obj.char == 's'
                stroke1 = stroke(arc(pi/5, pi-pi/5, a/2, a/2, a/2+b, 1));
                stroke3 = stroke(arc(2*pi-pi/5, pi+pi/5, a/2, a/2, a/2+b, 0));
                h = stroke1.getHeight();
                w = stroke1.getWidth();
                
                stroke2 = stroke(strokeline(b/4, a+b-h, w+b/4, b+h));
                
                output = [stroke1.getPoints();stroke2.getPoints();stroke3.getPoints()];
            end
            
            if obj.char == 't'
                stroke1 = stroke(strokeline(0, a+b, 0.6*a, a+b));
                output = stroke1.getPoints();
                w = stroke1.getWidth();
                stroke2 = stroke(strokeline(w/2, a+b+b, w/2, b));
                output = [output; liftPen; stroke2.getPoints()];
            end
            
            
            if obj.char == 'u'
                stroke2 = stroke(arc(pi+pi/6, 2*pi-pi/6, a/2, 0, 0, 1));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                stroke1 = stroke(strokeline(0, a+b, 0, b+h));
                s2p = obj.shiftVertical(stroke2.getPoints(),h+a/2);
                s2p = obj.shiftHorizontal(s2p, w/2);
                stroke3 = stroke(strokeline(w, a+b, w, b));
                output = [stroke1.getPoints(); s2p; liftPen; stroke3.getPoints()];
                
            end
            
            if obj.char == 'v'
                stroke1 = stroke(strokeline(0, a+b, 0.3*a, b));
                stroke2 = stroke(strokeline(0.3*a, b, 0.6*a, a+b));
                output = [stroke1.getPoints(); stroke2.getPoints()];
            end
            
            if obj.char == 'w'
                stroke1 = stroke(strokeline(0, a+b, 0.3*a, b));
                stroke2 = stroke(strokeline(0.3*a, b, 0.6*a, a+b));
                output = [stroke1.getPoints(); stroke2.getPoints()];
                output = [output; obj.shiftHorizontal(output, 0.6*a)];
            end
            
            if obj.char == 'x'
                stroke1 = stroke(strokeline(0, a+b, 0.6*a, b));
                stroke2 = stroke(strokeline(0, b, 0.6*a, a+b));
                output = [stroke1.getPoints(); liftPen; stroke2.getPoints()];
            end
            
            if obj.char == 'y'
                stroke2 = stroke(arc(pi+pi/6, 2*pi-pi/6, a/2, 0, 0, 1));
                h = stroke2.getHeight();
                w = stroke2.getWidth();
                stroke1 = stroke(strokeline(0, a+b, 0, b+h));
                s2p = obj.shiftVertical(stroke2.getPoints(),h+a/2);
                s2p = obj.shiftHorizontal(s2p, w/2);
                stroke3 = stroke(strokeline(w, a+b, w, b));
                output = [stroke1.getPoints(); s2p; liftPen; stroke3.getPoints()];
                
                
                stroke4 = stroke(arc(2*pi, pi+pi/5, b, 0, 0, 0));
                s4p = obj.scaleStroke(stroke4.getPoints(),w/b/2,1);
                s4p = obj.shiftHorizontal(s4p,w/2);
                s4p = obj.shiftVertical(s4p,b);
                output = [output; s4p];
                
            end
            
            if obj.char == 'z'
                stroke1 = stroke(strokeline(0, a+b, 0.6*a, a+b));
                stroke2 = stroke(strokeline(0.6*a, a+b,0, b));
                stroke3 = stroke(strokeline(0, b, 0.6*a, b));
                output = [stroke1.getPoints(); stroke2.getPoints(); stroke3.getPoints()];
            end
            
            
            
            if obj.char == ' '
                obj.maxWidth = 0.6*a;
                obj.points = [-1 -1 -1];
            else
                obj.maxWidth = obj.getMaxW(output);
                obj.points = output;
            end
            
            
        end
    end
end

