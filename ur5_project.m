% UR5 Stuff
rosshutdown;
rosinit('172.31.206.164', 11311); 
ur5 = ur5_interface_wsl();
% joints_home = [pi/3; -pi/4; -pi/3; pi/4; pi/2; pi/3]; % FIX
% joints_home = [-0.729 -0.204 -1.2081 -0.1584 -1.5709 -2.2998].';
joints_home = [-0.729 -0.6492 -0.5577 -0.3638 -1.5708 -2.2998].';
% joints_home = [-1.0053 -1.1310 -0.5655 0 -1.57 -1.4451].';
tool2gripper = ROTZ(pi/2, true);
tool2gripper(3,4) = 0.13; 
offset = [0 -pi/2 0 -pi/2 0 0]';
% joints_home = ur5.home;
home = ur5FwdKin(joints_home);

ur5.move_joints(joints_home, 10);
pause(10); 

% Correct frames to track gripper:
gst_1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1]/tool2gripper;
gst_2 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1]/tool2gripper;

% Workflow explanation:
% Robot at home configuration. 
% Robot moves so the EE frame goes to pose directly above start location.
% Robot moves straight down to the start location.
% Robot moves back to initial location (above inital).
% Robot moves above the target location. 
% Robot moves down to end location.
% Robot moves back to above the target location.

type = input("Choose algorithm type: IK, DK, or gradient: \n"); 

% Wait for button press before beginning movement:
disp("Click figure to start motion.");
w = waitforbuttonpress;

% Run corresponding algorithm to input string:
if strcmpi(type, "IK")
    err = run_ur5IKcontrol(gst_1, gst_2, home, ur5); 
elseif strcmpi(type, "DK")
    K = 1.5;
    err = run_ur5RRcontrol(gst_1, gst_2, K, home, ur5);
elseif strcmpi(type, "gradient")
    K = 0.5;
    err = run_ur5TJcontrol(gst_1, gst_2, K, home, ur5); 
else
    disp("Invalid algorithm type. Options are: 'IK, DK, or gradient.'");
end

disp("Error: \n");
disp(err);
    
