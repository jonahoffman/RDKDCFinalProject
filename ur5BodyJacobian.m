function Jb = ur5BodyJacobian(q)
% Calculat the UR5 Body Jacobian based on the given joint config. 

% Link lengths and joint positions
l1 = 0.425;
l2 = 0.392;
l3 = 0.1093;
l4 = 0.09475;
l5 = 0.0825;
l0 = 0.0892;
a1 = q(1);
a2 = q(2);
a3 = q(3);
a4 = q(4);
a5 = q(5);
a6 = q(6);

% Initialize Jb
Jb = zeros(6,6);

% Fill in Jb matrix based on formula from fkinjb.nb Mathematica notebook
Jb(1,1) = sin(a2)*(-cos(a6)*((l5 + l3*cos(a5))*sin(a3 + a4) + (l4*cos(a3 + a4) + l2*sin(a3))*sin(a5)) - cos(a3 + a4)*(l3 + l5*cos(a5))*sin(a6)) + cos(a2)*(cos(a3 + a4)*(l5 + l3*cos(a5))*cos(a6) + cos(a6)*(l1 + l2*cos(a3) - l4*sin(a3 + a4))*sin(a5) - (l3 + l5*cos(a5))*sin(a3 + a4)*sin(a6));
Jb(1,2) = cos(a5)*cos(a6)*(l4 - l2*sin(a4) - l1*sin(a3 + a4)) - (l2*cos(a4) + l1*cos(a3 + a4) + l5*sin(a5))*sin(a6);
Jb(1,3) = cos(a5)*cos(a6)*(l4 - l2*sin(a4)) - (l2*cos(a4) + l5*sin(a5))*sin(a6);
Jb(1,4) = l4*cos(a5)*cos(a6) - l5*sin(a5)*sin(a6);
Jb(1,5) = -l5*cos(a6);
Jb(2,1) = -cos(a3)*(l3 + l5*cos(a5))*cos(a6)*sin(a2 + a4) + ((l5 + l3*cos(a5))*sin(a3)*sin(a2 + a4) + (-cos(a2)*(l1 + l2*cos(a3)) + l2*sin(a2)*sin(a3) + l4*cos(a3)*sin(a2 + a4))*sin(a5))*sin(a6) - cos(a2 + a4)*((l3 + l5*cos(a5))*cos(a6)*sin(a3) + (cos(a3)*(l5 + l3*cos(a5)) - l4*sin(a3)*sin(a5))*sin(a6));
Jb(2,2) = -cos(a6)*(l2*cos(a4) + l1*cos(a3 + a4) + l5*sin(a5)) + cos(a5)*(-l4 + l2*sin(a4) + l1*sin(a3 + a4))*sin(a6);
Jb(2,3) = -cos(a6)*(l2*cos(a4) + l5*sin(a5)) + cos(a5)*(-l4 + l2*sin(a4))*sin(a6);
Jb(2,4) = -l5*cos(a6)*sin(a5) - l4*cos(a5)*sin(a6);
Jb(2,5) = l5*sin(a6);
Jb(3,1) = cos(a5)*(l1*cos(a2) + l2*cos(a2 + a3) - l4*sin(a2 + a3 + a4)) - l3*cos(a2 + a3 + a4)*sin(a5);
Jb(3,2) = (-l4 + l2*sin(a4) + l1*sin(a3 + a4))*sin(a5);
Jb(3,3) = (-l4 + l2*sin(a4))*sin(a5);
Jb(3,4) = -l4*sin(a5);
Jb(4,1) = cos(a5)*cos(a6)*sin(a2 + a3 + a4) + cos(a2 + a3 + a4)*sin(a6);
Jb(4,2) = cos(a6)*sin(a5);
Jb(4,3) = cos(a6)*sin(a5);
Jb(4,4) = cos(a6)*sin(a5);
Jb(4,5) = -sin(a6);
Jb(5,1) = cos(a2 + a3 + a4)*cos(a6) - cos(a5)*sin(a2 + a3 + a4)*sin(a6);
Jb(5,2) = -sin(a5)*sin(a6);
Jb(5,3) = -sin(a5)*sin(a6);
Jb(5,4) = -sin(a5)*sin(a6);
Jb(5,5) = -cos(a6);
Jb(6,1) = -sin(a2 + a3 + a4)*sin(a5);
Jb(6,2) = cos(a5);
Jb(6,3) = cos(a5);
Jb(6,4) = cos(a5);
Jb(6,6) = 1;

end

