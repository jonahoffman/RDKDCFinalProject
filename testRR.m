rosshutdown;
rosinit('172.18.207.239', 11311); 
ur5 = ur5_interface_wsl();
tool2gripper = ROTZ(pi/2, true);
tool2gripper(3,4) = 0.13; 
gst_1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];

frame = tf_frame("base_link", "gripper_pls", gst_1);
gst_j1 = [pi/3; -pi/4; -pi/3; pi/4; pi/2; pi/6];
gst_j2 = [pi/3; -pi/3; -pi/6; pi/4; pi/3; pi/6];
home = ur5.home - [ 0 pi/2 pi/2 0 pi/2 0];
% gst_1 = ur5FwdKin(gst_j1);
gst_2 = ur5FwdKin(gst_j1);
ur5.move_joints(gst_j1,10);
pause(10);
K = 1.5;
% ur5RRcontrol(gst_2, K, ur5);

correct_trans = gst_1/(tool2gripper); 
% err = ur5RRcontrol(correct_trans, K, ur5);
err = ur5IKcontrol(correct_trans, ur5);