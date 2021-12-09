function gst = ur5FwdKin(q)
% Calculate the forward kinematics of the UR5 robot based on given joint
% config. 

% Link lengths and joint values
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

% Initialize transformation
gst = zeros(4,4);

% Fill in gst matrix based on formula from fkinjb.nb Mathematica notebook
gst(4,4) = 1;
gst(1,1) = -cos(a6)*(cos(a1)*cos(a2 + a3 + a4)*cos(a5)+sin(a1)*sin(a5)) + cos(a1)*sin(a2 + a3 + a4)*sin(a6);
gst(1,2) = sin(a1)*sin(a5)*sin(a6)+cos(a1)*(cos(a6)*sin(a2 + a3 + a4) + cos(a2 + a3 + a4)*cos(a5)*sin(a6));
gst(1,3) = -cos(a5)*sin(a1)+cos(a1)*cos(a2 + a3 + a4)*sin(a5);
gst(1,4) = -((l3 + l5*cos(a5))*sin(a1)) + cos(a1)*(l1*cos(a2) + l2*cos(a2 + a3) - l4*sin(a2 + a3 + a4) + l5*cos(a2 + a3 + a4)*sin(a5));
gst(2,1) = -cos(a2 + a3 + a4)*cos(a5)*cos(a6)*sin(a1) + cos(a1)*cos(a6)*sin(a5) + sin(a1)*sin(a2 + a3 + a4)*sin(a6);
gst(2,2) = cos(a6)*sin(a1)*sin(a2 + a3 + a4) + (cos(a2 + a3 + a4)*cos(a5)*sin(a1) - cos(a1)*sin(a5))*sin(a6);
gst(2,3) = cos(a1)*cos(a5) + cos(a2 + a3 + a4)*sin(a1)*sin(a5);
gst(2,4) = cos(a1)*(l3 + l5*cos(a5)) + sin(a1)*(l1*cos(a2) + l2*cos(a2 + a3) - l4*sin(a2 + a3 + a4) + l5*cos(a2 + a3 + a4)*sin(a5));
gst(3,1) = cos(a5)*cos(a6)*sin(a2 + a3 + a4) + cos(a2 + a3 + a4)*sin(a6);
gst(3,2) = cos(a2 + a3 + a4)*cos(a6) - cos(a5)*sin(a2 + a3 + a4)*sin(a6);
gst(3,3) = -sin(a2 + a3 + a4)*sin(a5);
gst(3,4) = l0 - l4*cos(a2 + a3 + a4) - l1*sin(a2) - l2*sin(a2 + a3) - l5*sin(a2 + a3 + a4)*sin(a5);

end

