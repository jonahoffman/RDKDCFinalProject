% UR5 Stuff
ur5 = ur5_interface();
joints_home = [0 0 0 0 0 0]; % FIX
home = ur5FwdKin(joints_home);
ur5.move_joints(home, 10);
pause(10); 

gst_1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
gst_2 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];

% Robot at home configuration. 
% Robot moves so the EE frame goes to pose directly above start location.
% Robot moves straight down to the start location.
% Robot moves back to initial location (above inital).
% Robot moves above the target location. 
% Robot moves down to end location.
% Robot moves back to above the target location.

type = input("Choose algorithm type: IK, DK, or gradient"); 

if strcmpi(type, "IK")
    ur5IKcontrol(gst_1, gst_2, home, ur5); 
elseif strcmpi(type, "DK")
    K = 0.25;
    run_ur5RRcontrol(gst_1, gst_2, K, home, ur5);
elseif strcmpi(type, "gradient")
    K = 0.25;
    run_ur5TJcontrol(gst_1, gst_2, K, home, ur5); 
else
    disp("Invalid algorithm type. Options are: 'IK, DK, or gradient.'");
end
    
