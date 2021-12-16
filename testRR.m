rosshutdown;
rosinit('172.31.206.164', 11311); 
ur5 = ur5_interface_wsl();
tool2gripper = ROTZ(pi/2, true);
tool2gripper(3,4) = 0.13; 
gst_1 = [0 -1 0 0.2; -1 0 0 -0.1; 0 0 -1 0.55; 0 0 0 1];
home = [-0.729 -0.204 -1.2081 -0.1584 -1.5709 -2.2998].';
% home = [-0.729 -0.6492 -0.5577 -0.3638 -1.5708 -2.2998].';
frame = tf_frame("base_link", "gripper_pls", gst_1);
gst_j1 = [pi/3; -pi/6; -pi/4; pi/4; pi/2; pi/6];
gst_j2 = [pi/3; -pi/3; -pi/6; pi/4; pi/3; pi/6];
offset = [0 -pi/2 pi/6 -pi/2 pi/6 0]';
% gst_1 = ur5FwdKin(gst_j1);
gst_2 = ur5FwdKin(gst_j2);
ur5.move_joints(home, 10);
pause(10);
K = 1.0;
% ur5RRcontrol(gst_2, K, ur5);

correct_trans = gst_2/(tool2gripper); 
ur5RRcontrol(correct_trans, K, ur5);
% ur5IKcontrol(correct_trans, ur5);