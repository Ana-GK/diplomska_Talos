% necessary functions and Excels:

% default_pose.xlsx
% compute_cart_positions.m
% compute_joint_positions.m
% error_codes.m
% tf_ft2base.m
% move.m

% Comments (changes):

    % cartesic limits are calculated using default position read from an Excel
    % file default_pose.xslx

    % Talos moves in any direction (not just x or y or z axis) if the apllied force
    % is greater than lowest_F_value

    % Coponent of the force needs to be larger than
    % lowest_F_value_component for the move to contain that direction; size
    % of the move in that direction depends on the value of the component

    % Size of the force is calculated only after the components of the force
    % lower than lowest_F_value_component have been excluded.

    % IK and FK computing functions are seperate from the main script


%% Program for force dependent movement of Talos (INTEGRATES FORCES)

% Speed of the movement depends on the size of the force applied.

% New joint positions are calculated in every iteration of the while loop from
% current joint positions - which are actually previously calculated joint
% positions, so that it doesn't read joint states in every loop

% Current cartesic position is calculated and saved in every iteration (from
% the "integral")
% (forwards kinematics - {joints to x,y,z} - is calculated only once)

% If 90% of the joint limit is reached while Talos is moving, the movement
% is stopped
% Talos also has defined work space in cartesic coordinates in which it can
% move.


% DIRECTION OF FORCES
% x direction: Talos moves forwards(+) and backwards(-)
% y direction: Talos moves right(+) and left(-)
% z direction: Talos moves up(+) and down(-)

%%%%%%%%%%%%%%%%%%%%%%%%%
% DESIRED MOVEMENT; direction of aplied force to: L_GRIPPER and R_GRIPPER:

% FORWARD L:+x, R:+x
% BACK L:-x, R-x:
% RIGHT L:+y, R:+y
% LEFT L:-y, R:-y
% UP L:+z, R:+z
% DOWN L:-z, R:-z
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize ROS

clear all;

rosshutdown;
pause(1);

rosinit;
pause(1);

global pub_Lleg;
global msg_Lleg;
global pub_Rleg;
global msg_Rleg;
global pub_Larm;
global msg_Larm;
global pub_Rarm;
global msg_Rarm;
global pub_torso;
global msg_torso;
global pub_head;
global msg_head;

global msgP_Lleg;
global msgP_Rleg;
global msgP_Larm;
global msgP_Rarm;
global msgP_torso;
global msgP_head;

[pub_Lleg,msg_Lleg] = rospublisher('/left_leg_controller/command','trajectory_msgs/JointTrajectory');
[pub_Rleg,msg_Rleg] = rospublisher('/right_leg_controller/command','trajectory_msgs/JointTrajectory');
[pub_Larm,msg_Larm] = rospublisher('/left_arm_controller/command','trajectory_msgs/JointTrajectory');
[pub_Rarm,msg_Rarm] = rospublisher('/right_arm_controller/command','trajectory_msgs/JointTrajectory');
[pub_torso,msg_torso] = rospublisher('/torso_controller/command','trajectory_msgs/JointTrajectory');
[pub_head,msg_head] = rospublisher('/head_controller/command','trajectory_msgs/JointTrajectory');

msgP_Lleg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Rleg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Larm = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_Rarm = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_torso = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgP_head = rosmessage('trajectory_msgs/JointTrajectoryPoint');

msg_Lleg.JointNames = {'leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint'};
msg_Rleg.JointNames = {'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint'};
msg_Larm.JointNames = {'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'};
msg_Rarm.JointNames = {'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'};
msg_torso.JointNames = {'torso_1_joint', 'torso_2_joint'};
msg_head.JointNames = {'head_1_joint', 'head_2_joint'};

global sub_l;
global sub_r;
global msg_l;
global msg_r;

disp("Global variables initialized")

pause(1);

sub_r = rossubscriber('/right_wrist_ft');
pause(2);
msg_r = receive(sub_r, 5);

disp("Received data for right wrist")

sub_l = rossubscriber('/left_wrist_ft');
pause(2);
msg_l = receive(sub_l, 5);

disp("Received data for left wrist")


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The controll window

global swi1;

fig = uifigure;
swi1 = uiswitch(fig);
swi1.Position = [80 350 45 20];
swi1.Value = 'On';
uilabel(fig,'Position',[180 350 200 20],'Text','Stop the simulation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Error codes - CREATE DICTIONARIES TO LOOKUP FK AND IK ERRORS

[d_fk,d_ik] = error_codes();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read initial joint states

global sub_states;
global msg_states;

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 1); % 1 is timeout in seconds

% global initial_joint_positions;
% global initial_joint_names;

initial_joint_positions = msg_states.Position; % Save initial joint positions
initial_joint_names = msg_states.Name; % Save joint names

disp(initial_joint_positions)

% Divide joint positions in groups by limbs

LArm = initial_joint_positions(1:7);
RArm = initial_joint_positions(8:14);

LLeg = initial_joint_positions(19:24);
RLeg = initial_joint_positions(25:30);

Head = initial_joint_positions(17:18);
Torso = initial_joint_positions(31:32);

% Save the 7th joint, the value is used after the inverse kinematics, to
% correct the angle of the 7th joint (it stays locked on ther same value)

initial_joint_position_arm_left_7_joint = LArm(7);
initial_joint_position_arm_right_7_joint = RArm(7);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forces constants

% Lowest value of applied force in one direction for Talos to move
% Za vsako smer posebej
lowest_F_value_component = 5;
% Lowest value of applied force for Talos to move
% (za seštevek sil)
lowest_F_value = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint limits

global lower_limit;
global upper_limit;
global new_lower_limit;
global new_upper_limit;

lower_limit = [-1.57,0.01,-2.43,-2.23,-2.51,-1.37,-0.68,-0.79,-2.87,-2.43,-2.23,-2.51,-1.37,-0.68,-0.96,-0.96,-0.21,-1.31,-0.35,-0.52,-2.10,-0.00,-1.27,-0.52,-1.57,-0.52,-2.10,0.00,-1.27,-0.52,-1.26,-0.23];
upper_limit = [0.79,2.87,2.43,0.00,2.51,1.37,0.68,1.57,-0.01,2.43,0.00,2.51,1.37,0.68,0.00,0.00,0.79,1.31,1.57,0.52,0.70,2.62,0.68,0.52,0.35,0.52,0.70,2.62,0.68,0.52,1.26,0.73];

% new limit is 90% of actual limit
new_upper_limit = upper_limit - upper_limit*0.1;
new_lower_limit = lower_limit - lower_limit*0.1;

new_upper_limit = new_upper_limit';
new_lower_limit = new_lower_limit';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cart limits

% upper limit is limit in the positive direction of the axis
% lower limit is the limit in the negative direction of the axis

file_name = 'default_pose.xlsx'; % file with default position
file_path = '/home/cobotat4/Documents/MATLAB/Talos_Kuka_project/';
file = strcat(file_path, file_name);

global x_up_limit
global x_low_limit
global y_up_limit
global y_low_limit
global z_up_limit
global z_low_limit

% Read table of cartesic positions from Excel file
default_cart_positions_table = readtable(file,'Sheet',1,'Range','C1:F8');
% Select only 2nd and 4th column of the table and transform them into array
default_cart_positions = table2array(default_cart_positions_table(:,2:2:4));
% Transponse
default_cart_positions = default_cart_positions';

x_up_limit = default_cart_positions(1,1) + 0.10;
x_low_limit = default_cart_positions(1,1) - 0.07;

y_up_limit(1) = default_cart_positions(1,2) + 0.10;
y_up_limit(2) = default_cart_positions(2,2) + 0.10;
y_low_limit(1) = default_cart_positions(1,2) - 0.10;
y_low_limit(2) = default_cart_positions(2,2) - 0.10;

z_up_limit = default_cart_positions(1,3) + 0.15;
z_low_limit = default_cart_positions(1,3) - 0.10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forward kinematics

[link_names, coordinates] = compute_cart_positions(initial_joint_positions, initial_joint_names,d_fk);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global num_limbs
num_limbs = 2; % Left and right arm

initial_cart_position = [];

 % Cartesic positions for both arms (left arm has first coordinate 1, right arm has first coordinate 2)

for i=1:num_limbs
    initial_cart_position(i,1) = coordinates(7,i).Pose.Position.X;
    initial_cart_position(i,2) = coordinates(7,i).Pose.Position.Y;
    initial_cart_position(i,3) = coordinates(7,i).Pose.Position.Z;
    initial_cart_position(i,4) = coordinates(7,i).Pose.Orientation.X;
    initial_cart_position(i,5) = coordinates(7,i).Pose.Orientation.Y;
    initial_cart_position(i,6) = coordinates(7,i).Pose.Orientation.Z;
    initial_cart_position(i,7) = coordinates(7,i).Pose.Orientation.W;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Offseti

% msg_r = receive(sub_r, 5);
% msg_l = receive(sub_l, 5);
% 
% Fl = msg_l.Wrench.Force;
% Tl = msg_l.Wrench.Torque;
% 
% Fr = msg_r.Wrench.Force;
% Tr = msg_r.Wrench.Torque;
% 
% Ft_l = [Fl.X, Fl.Y, Fl.Z, Tl.X, Tl.Y, Tl.Z];
% Ft_r= [Fr.X, Fr.Y, Fr.Z, Tr.X, Tr.Y, Tr.Z];

Rt_l = initial_cart_position(1,4:7);
Rt_r = initial_cart_position(2,4:7);
% 
% [mass_l,COM_l,Off_l]=load_est(Ft_l,Rt_l)
% [mass_r,COM_r,Off_r]=load_est(Ft_r,Rt_r)
% 
% disp(Off_l(1)-offset_LX)
% disp(Off_l(2)-offset_LY)
% disp(Off_l(3)-offset_LZ)
% 
% disp(Off_r(1)-offset_RX)
% disp(Off_r(2)-offset_RY)
% disp(Off_r(3)-offset_RZ)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize all other variables for the loop

i = 0; % for counting iterations

K = 0.00015; % constant for integral

current_cart_position = initial_cart_position; % for the first iteration of the while loop

% all_cart_positions_L = [];
% all_cart_positions_R = [];
% all_read_joints = [];
initial_joints = [LArm' RArm'];
% With frequency 10 HZ, for 5 min
F_values_for_plot = zeros(3000,3);
array_time = zeros(3000,1);
loop_count = 0;
time_of_loops = zeros(3000,1);
time_of_parts = zeros(3000,8);
quat=zeros(2,4);

previous_joints = msg_states; % initialize variable previous_joints
new_joints = previous_joints; % set new_joints to previous joints to avoid an error when
% a force exists already at the start of the loop

% r = rosrate(50);

% Offset for weight of a gripper
F_off_z = 14.61;

% inicialize transform tree
tftree = rostf;
pause(2)

% start timer
tic;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while strcmp(swi1.Value,'On') % While the button is on

    loop_count = loop_count+1;
    time1 = toc;
    time_of_loops(loop_count) = time1;

    msg_r = receive(sub_r, 1); % 1 is timeout in seconds
    msg_l = receive(sub_l, 1); % 1 is timeout in seconds


    Fl = msg_l.Wrench.Force;
    Tl = msg_l.Wrench.Torque;
    
    Fr = msg_r.Wrench.Force;
    Tr = msg_r.Wrench.Torque;
    
    FL = [Fl.X, Fl.Y, Fl.Z];
    FR = [Fr.X, Fr.Y, Fr.Z];
    
    % get current quaternions from tftree
    trans_left = getTransform(tftree,'base_link','wrist_left_ft_link');
    q_left = trans_left.Transform.Rotation;
    quat_left = [q_left.W q_left.X q_left.Y q_left.Z];
    % get current quaternions from tftree
    trans_right = getTransform(tftree,'base_link','wrist_right_ft_link');
    q_right = trans_right.Transform.Rotation;
    quat_right = [q_right.W q_right.X q_right.Y q_right.Z];

    % calculate force in the frame of base_link
    F_l = tf_ft2base(quat_left,FL);
    F_r = tf_ft2base(quat_right,FR);


    F_x_l = F_l(1);
    F_y_l = F_l(2);
    F_z_l = F_l(3)+F_off_z;

    F_x_r = F_r(1);
    F_y_r = F_r(2);
    F_z_r = F_r(3)+F_off_z;

    F_x = F_x_l + F_x_r;
    F_y = F_y_l + F_y_r;
    F_z = F_z_l + F_z_r;


    time15 = toc;

%     F_x = -F_z_l - F_z_r; % if F_x is + Talos moves forward, if F_x is - Talos moves back
%     F_y = -F_x_l + F_x_r; % if F_y is + Talos moves right, if F_y is - Talos moves left
%     F_z = F_y_l + F_y_r; % if F_z is + Talos moves up, if F_z is - Talos moves down

    fprintf('Fx: %f, Fy: %f, F_z: %f, ',F_x,F_y,F_z)

    F_values_for_plot(loop_count,:) = [F_x F_y F_z];

    time2 = toc; % got forces
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Set force components lower than lowest_F_value_component to 0
    
    forces_Talos = [F_x F_y F_z];

    for i=1:length(forces_Talos)
        if abs(forces_Talos(i)) < lowest_F_value_component
            forces_Talos(i)=0;
        end
    end

    force_size = sqrt(forces_Talos(1)^2 + forces_Talos(2)^2+ forces_Talos(3)^2);
    
    % Set all the force components to 0 if the force is lower than
    % lowest_F_value

    if ~(force_size > lowest_F_value)
        forces_Talos = [0 0 0];
    end

    time3 = toc; % calculated forces
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % Positions for when all forces are lower than lowest_F_value (same as
    % previous position)
    LArm = previous_joints.Position(1:7);
    RArm = previous_joints.Position(8:14);

%     read_joints = [LArm' RArm'];
%     all_read_joints = [all_read_joints; read_joints];

    new_cart_position = current_cart_position;


    if any(forces_Talos) % if any of the force components is not 0
        for i=1:num_limbs
            new_cart_position(i,1) = new_cart_position(i,1) + forces_Talos(1)*K;
            new_cart_position(i,2) = new_cart_position(i,2) + forces_Talos(2)*K;
            new_cart_position(i,3) = new_cart_position(i,3) + forces_Talos(3)*K;
        end
    else
        new_joints = previous_joints;
    end

    % Pogledam če je pozicija Talosa v kartezičnih koordinatah izven
    % delovnega prostora
    % V primeru gre čez mejo, new_cart_position spet postavim na
    % current_cart_position
    % stop je 1, če nova pozicija presega meje

    stop = inside_work_zone(new_cart_position);
    
    if stop == 1
        new_cart_position = current_cart_position;
    end


%     disp("Force x:")
%     disp(F_x)
%     disp("Force y:")
%     disp(F_y)
%     disp("Force z:")
%     disp(F_z)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % x direction (forward, back) - both arms together

    % y direction (left, right) - both arms together

    % z direction (up, down) - both arms together

    time35 = toc;

    if any(forces_Talos) && ~stop
        [LArm, RArm] = compute_joint_positions(new_cart_position,previous_joints,d_ik); % delta_x is the move size
        force_index = find(forces_Talos); % finds index of each non-zero force
        fprintf("Moving in ")
        if any(force_index == 1)
            fprintf("x")
        end
        if any(force_index == 2)
            fprintf("y")
        end
        if any(force_index == 3)
            fprintf("z")
        end
        fprintf(" direction.\n")
        change=1;
    else
        change=0;
    end

    
    % Add new joint positions to array new_joints

    new_joints.Position(1:7) = LArm;
    new_joints.Position(8:14) = RArm;


    % Checks if limit was reached and returns the name of the joint that
    % has reached the limit
    % stop_movement is 1 if limit was reached

    
    [stop_movement] = is_limit_reached(new_joints.Position,new_joints.Name);
    

    % Change 7th joint position to initial value

    LArm(7) = initial_joint_position_arm_left_7_joint;
    RArm(7) = initial_joint_position_arm_right_7_joint;
    
    time4 = toc; % all calculations

    % Publish new joint positions

    if stop_movement ~= 1 % stop_movement is 1 if limit was reached

        dT1 = 0.5;

        move(LLeg,RLeg,LArm,RArm,Head,Torso,dT1)

        current_cart_position = new_cart_position;
    else
        disp("Stopping this move.")
%         quasi_integral = all_q_integrals(end-1);
        new_cart_position = current_cart_position; % new_cart_positions are set to previous cartesic positions so that it doesn't save unused positions
        new_joints = previous_joints;
    end

    time5 = toc;

    previous_joints = new_joints;

    [~,coordinates]=compute_cart_positions(new_joints.Position,new_joints.Name,d_fk);
    for i=1:num_limbs
        quat(i,1) = coordinates(7,i).Pose.Orientation.X;
        quat(i,2) = coordinates(7,i).Pose.Orientation.Y;
        quat(i,3) = coordinates(7,i).Pose.Orientation.Z;
        quat(i,4) = coordinates(7,i).Pose.Orientation.W;
    end

    Rt_l = quat(1,1:4);
    Rt_r = quat(2,1:4);
    

%     all_cart_positions_L = [all_cart_positions_L; new_cart_position(1,:)];
%     all_cart_positions_R = [all_cart_positions_R; new_cart_position(2,:)];

%     time = r.TotalElapsedTime;
%     array_time(loop_count) = time;
%     fprintf('Time Elapsed: %f\n',time)

    time_of_parts(loop_count,:) = [change time1 time15 time2 time3 time35 time4 time5];

%     waitfor(r);


end

% Display changes in joint positions that occured during the program run

% disp("RArm")
% 
% disp("LArm")
% 
% disp("Torso")

% draw_forces_and_movement(F_values_for_plot,all_cart_positions_L,all_cart_positions_R,array_time)

% disp(all_read_joints)
% disp(initial_joints)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS

function [stop] = inside_work_zone(cart_position)

    global x_up_limit
    global x_low_limit
    global y_up_limit
    global y_low_limit
    global z_up_limit
    global z_low_limit

    global num_limbs

    for i=1:num_limbs
        if ~all(cart_position(i,1:3) < [x_up_limit,y_up_limit(i),z_up_limit])
            fprintf("Upper limit of work space reached.")
            stop = 1;
        elseif ~all(cart_position(i,1:3) > [x_low_limit,y_low_limit(i),z_low_limit])
            fprintf("Lower limit of work space reached.")
            stop = 1;
        else
            stop = 0;
        end
    end
end

function [stop_movement] = is_limit_reached(joint_positions, joint_names)

    global new_upper_limit;
    global new_lower_limit;


    if all(joint_positions < new_upper_limit) && all(joint_positions > new_lower_limit)
        disp('all joints within limit')
        stop_movement = 0;
    else
        for i=1:32
            if joint_positions(i) > new_upper_limit(i)
                fprintf('Joint: %s (index %d) has reached 90%% of upper limit\n',string(joint_names(i)),i);
                stop_movement = 1;
            end
            if joint_positions(i) < new_lower_limit(i)
                fprintf('Joint: %s (index %d) has reached 90%% of lower limit\n',string(joint_names(i)),i);
                stop_movement = 1;
            end
        end
    end
end
    

