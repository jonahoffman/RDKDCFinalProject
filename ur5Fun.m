function [gst,H1,H2,H3,H4,H5,H6,s1,s2,s3,s4,s5,s6,gst0] = ur5Fun(q)

rotscrew = @(q, w) [-cross(w,q); w];

l1 = 0.425;
l2 = 0.392;
l3 = 0.1093;
l4 = 0.09475;
l5 = 0.0825;
l0 = 0.0892;

e1 = [1 0 0]';
e2 = [0 1 0]';
e3 = [0 0 1]';

w1 = e3;
w2 = e2;
w3 = e2;
w4 = e2;
w5 = -e3;
w6 = e2;
q1 = [0 0 0]';
q2 = [0 0 l0]';
q3 = [l1 0 l0]';
q4 = [l1+l2 0 l0]';
q5 = [l1+l2 l3 0]';
q6 = [l1+l2 0 l0-l4]';

gst0 = [[-1 0 0; 0 0 1; 0 1 0] [l1+l2 l3+l5 l0-l4]'; zeros(1,3) 1];

s1 = rotscrew(q1, w1);
s2 = rotscrew(q2, w2);
s3 = rotscrew(q3, w3);
s4 = rotscrew(q4, w4);
s5 = rotscrew(q5, w5);
s6 = rotscrew(q6, w6);


H1 = twistExp(s1,q(1));
H2 = twistExp(s2,q(2));
H3 = twistExp(s3,q(3));
H4 = twistExp(s4,q(4));
H5 = twistExp(s5,q(5));
H6 = twistExp(s6,q(6));


gst = H1*H2*H3*H4*H5*H6*gst0;


end

