function g = twistExp(s,t)

mexp = @(A) eye(3)+A+(A^2)/2+(A^3)/6+(A^4)/24;

v = s(1:3,1);
w = s(4:6,1);

R = mexp(skew3(w)*t);
p = (eye(3)-R)*cross(w,v) + w*w'*v*t;

g = [R p; 0 0 0 1];

end

