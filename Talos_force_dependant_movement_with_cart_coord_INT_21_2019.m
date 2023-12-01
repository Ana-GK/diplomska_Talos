% FOR MATLAB 2019b

% necessary functions and Excels:

% default_pose_grippers.xlsx
% compute_cart_positions.m
% compute_joint_positions.m
% tf_ft2base.m
% change_Sensor_frequency_2019.m

% you can start from default position or current position (by changing the
% variable GO_TO_DEFAULT_POSE (1 means it goes to default pose, 0 means it
% stays in current position)
% limits in cartesian coordinates stay the same no matter where you start

% Other comments:

    % Include force sensors

    % cartesic limits are calculated using default position read from an Excel
    % file default_pose_grippers.xslx

    % Talos moves in any direction (not just x or y or z axis) if the apllied force
    % is greater than lowest_F_value

    % A component of the force needs to be larger than
    % lowest_F_value_component for the move to contain that direction; size
    % of the move in that direction depends on the value(size) of the component


%% Program for force dependent movement of Talos (INTEGRATES FORCES)

% Moves using whole_body_control

% Speed of the movement depends on the size of the force applied.

% Current cartesic position is calculated and saved in every iteration (from
% the "integral")

% Joint limits are implemented in wbc controller (not yet tested)

% Talos has defined work space in cartesic coordinates (calculated from the
% default pose) in which it can move.


% DIRECTION OF FORCES
% x direction: Talos moves forwards(+) and backwards(-)
% y direction: Talos moves right(+) and left(-)
% z direction: Talos moves up(+) and down(-)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize ROS

close all;
clear all;

rosshutdown;
pause(1);

rosinit;
pause(1);

change_ft_sensor_frequency_2019()
pause(1)

GO_TO_DEFAULT_POSE = 1; % 1 - go to default pose, 0 - start from current pose

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

global num_limbs
num_limbs = 2;
global display_frequency
display_frequency = 50;
global loop_count

[pubL,msgL]=rospublisher('/whole_body_kinematic_controller/gripper_left','geometry_msgs/PoseStamped');
[pubR,msgR]=rospublisher('/whole_body_kinematic_controller/gripper_right','geometry_msgs/PoseStamped');


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

% [d_fk,d_ik] = error_codes();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forces constants

% Lowest value of applied force in one direction for Talos to move
% (za vsako smer posebej)
lowest_F_value_component = 2;
% Lowest value of applied force for Talos to move
% (za seštevek sil)
lowest_F_value = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Cart limits

% upper limit is limit in the positive direction of the axis
% lower limit is the limit in the negative direction of the axis

file_name = 'default_pose_grippers.xlsx';
file_path = '/home/cobotat4/Documents/MATLAB/Talos_Kuka_project/';
file = strcat(file_path, file_name);

global x_up_limit
global x_low_limit
global y_up_limit
global y_low_limit
global z_up_limit
global z_low_limit

% Read table of cartesic positions from Excel file
default_cart_positions = readmatrix(file,'Sheet',1,'Range','B2');
% Transponse
default_cart_positions = default_cart_positions';

if GO_TO_DEFAULT_POSE == 1
    initial_cart_position = default_cart_positions;
end
publish_cartesic_positions(initial_cart_position,pubL,msgL,pubR,msgR);
disp('GOING TO DEFAULT POSITION.')
pause(2)

x_up_limit = default_cart_positions(1,1) + 0.10;
x_low_limit = default_cart_positions(1,1) - 0.07;

y_up_limit(1) = default_cart_positions(1,2) + 0.10;
y_up_limit(2) = default_cart_positions(2,2) + 0.10;
y_low_limit(1) = default_cart_positions(1,2) - 0.10;
y_low_limit(2) = default_cart_positions(2,2) - 0.10;

z_up_limit = default_cart_positions(1,3) + 0.15;
z_low_limit = default_cart_positions(1,3) - 0.10;

% x_up_limit = default_cart_positions(1,1) + 0.30;
% x_low_limit = default_cart_positions(1,1) - 0.30;
% 
% y_up_limit(1) = default_cart_positions(1,2) + 0.30; % left
% y_up_limit(2) = default_cart_positions(2,2) + 0.30; % right
% y_low_limit(1) = default_cart_positions(1,2) - 0.30;
% y_low_limit(2) = default_cart_positions(2,2) - 0.30;
% 
% z_up_limit = default_cart_positions(1,3) + 0.30;
% z_low_limit = default_cart_positions(1,3) - 0.30;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inicialize transform tree
tftree = rostf;
pause(2)


% get current quaternions from tftree for left sensor
tf_left = getTransform(tftree,'base_link','gripper_left_base_link');
translation_left = tf_left.Transform.Translation;
orientation_left = tf_left.Transform.Rotation;

initial_cart_position(1,1) = translation_left.X;
initial_cart_position(1,2) = translation_left.Y;
initial_cart_position(1,3) = translation_left.Z;
initial_cart_position(1,4) = orientation_left.X;
initial_cart_position(1,5) = orientation_left.Y;
initial_cart_position(1,6) = orientation_left.Z;
initial_cart_position(1,7) = orientation_left.W;

% get current quaternions from tftree for left sensor
tf_right = getTransform(tftree,'base_link','gripper_right_base_link');
translation_right = tf_right.Transform.Translation;
orientation_right = tf_right.Transform.Rotation;

initial_cart_position(2,1) = translation_right.X;
initial_cart_position(2,2) = translation_right.Y;
initial_cart_position(2,3) = translation_right.Z;
initial_cart_position(2,4) = orientation_right.X;
initial_cart_position(2,5) = orientation_right.Y;
initial_cart_position(2,6) = orientation_right.Z;
initial_cart_position(2,7) = orientation_right.W;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize all other variables for the loop

i = 0; % for counting iterations

K = 0.00015; % constant for integral

current_cart_position = initial_cart_position; % for the first iteration of the while loop

F_values_for_plot = zeros(10000,3); % With frequency 10 HZ, for 5 min
loop_count = 0;
time_of_iterations = zeros(10000,1);
time_of_parts = zeros(10000,8);
time_of_ik = zeros(10000,1);
time_of_ik_parts = zeros(10000,5);
changes_in_loop = zeros(10000,1);
quat=zeros(2,4);
change = 0; % initialize variable for saving changes

% Initialize force sensors

N_data_frames = 3000;
FS_time = nan(N_data_frames,1); % the same time as for FP_data_Nth
FS_data = nan(N_data_frames,12);
FS_F = nan(N_data_frames,6);
FS_FT = nan(N_data_frames,12);

FT1 = ati_ft('Host','192.168.1.30');
% % % FT2 = ati_ft('Host','192.168.1.31');

% Offset for the weight of a gripper
F_off_z = 14.61;

% % inicialize transform tree
% tftree = rostf;
% pause(2)

% % create rosservice client
% ik_client = rossvcclient("/compute_ik",'moveit_msgs/GetPositionIK');
% msg_ik_l = rosmessage(ik_client);
% msg_ik_r = rosmessage(ik_client);

node = ros.Node('/testTime');
pause(0.5)
r = ros.Rate(node,100);

% start timer
tic;
reset(r);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while strcmp(swi1.Value,'On') % While the button is on
    
    loop_count = loop_count+1;
%     time1 = toc;
    t1=0;
    times=[0 0 0 0 0];

% % % % %     % get data from force sensor
% % % % %     FS_data(i,:) = [FT1.force' FT2.force'];
% % % % %     FS_F(i,:) = [FT1.F' FT2.F'];
    time1 = toc;
    % read the forces
%     msg_r = receive(sub_r, 1); % 1 is timeout in seconds
%     msg_l = receive(sub_l, 1); % 1 is timeout in seconds
    msg_l = sub_l.LatestMessage;
    msg_r = sub_r.LatestMessage;
    time2 = toc;

    % save output of the force sensor
    Fl = msg_l.Wrench.Force;
%     Tl = msg_l.Wrench.Torque;
    Fr = msg_r.Wrench.Force;
%     Tr = msg_r.Wrench.Torque;
    
    % create vector of the forces
    FL = [Fl.X, Fl.Y, Fl.Z];
    FR = [Fr.X, Fr.Y, Fr.Z];

    time3 = toc;

    % get current quaternions from tftree for left sensor
    trans_left = getTransform(tftree,'base_link','wrist_left_ft_link');
    q_left = trans_left.Transform.Rotation;
    quat_left = [q_left.W q_left.X q_left.Y q_left.Z];
    % get current quaternions from tftree for right sensor
    trans_right = getTransform(tftree,'base_link','wrist_right_ft_link');
    q_right = trans_right.Transform.Rotation;
    quat_right = [q_right.W q_right.X q_right.Y q_right.Z];

    time4 = toc;

    % calculate force in the frame of base_link
    F_l = tf_ft2base(quat_left,FL);
    F_r = tf_ft2base(quat_right,FR);

    time5 = toc;

% % % % % %     % for seperate arms
% % % % % %     F_x_l = F_l(1);
% % % % % %     F_y_l = F_l(2);
% % % % % %     F_z_l = F_l(3)+F_off_z;
% % % % % % 
% % % % % %     F_x_r = F_r(1);
% % % % % %     F_y_r = F_r(2);
% % % % % %     F_z_r = F_r(3)+F_off_z;

    % add up the forces
    F_x = F_l(1) + F_r(1);
    F_y = F_l(2) + F_r(2);
    F_z = F_l(3)+F_r(3) + 2*F_off_z;


%     time2 = toc;
    
    % display forces
    
    if mod(loop_count,display_frequency)==0
        fprintf('Fx: %f, Fy: %f, F_z: %f, ',F_x,F_y,F_z)
    end

    F_values_for_plot(loop_count,:) = [F_x F_y F_z];

%     time3 = toc; % got forces

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

%     time4 = toc; % calculated forces

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    new_cart_position = current_cart_position;


    if any(forces_Talos) % if any of the force components is not 0
        for i=1:num_limbs
            new_cart_position(i,1) = new_cart_position(i,1) + forces_Talos(1)*K;
            new_cart_position(i,2) = new_cart_position(i,2) + forces_Talos(2)*K;
            new_cart_position(i,3) = new_cart_position(i,3) + forces_Talos(3)*K;
        end
    else
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % x direction (forward, back) - both arms together

    % y direction (left, right) - both arms together

    % z direction (up, down) - both arms together

    % CALCULATE IK

%     time5 = toc;

    if any(forces_Talos) && ~stop

        a=tic;

        t1=toc(a);

        force_index = find(forces_Talos); % finds index of each non-zero force
        if mod(loop_count,display_frequency)==0
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
        end
        change=1;
    else
        change=0;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Publish new joint positions

    time6 = toc;
    publish_cartesic_positions(new_cart_position,pubL,msgL,pubR,msgR);
    time7 = toc;
    current_cart_position = new_cart_position;
%     time7 = toc;

%     time6 = toc;
    drawnow;
%     time7 = toc;
    changes_in_loop(loop_count) = [change];
    time_of_iterations(loop_count) = time1;
    time_of_parts(loop_count,:) = [change time1 time2 time3 time4 time5 time6 time7];
    time_of_ik(loop_count) = t1;
    waitfor(r);

%     disp(swi1.Value)
    if mod(loop_count,display_frequency)==0
    fprintf('\n')
    end

end

%%

time_of_ik=time_of_ik(1:loop_count-1);


time_loops = nonzeros(time_of_iterations);

i=1;

if time_of_parts(end,8)~=0
    time_of_parts(end+1,8)=[0 0 0 0 0 0 0 0];
end
while time_of_parts(i,8) ~= 0
    time_parts(i,:) = time_of_parts(i,:);
    i=i+1;
end

disp(time_loops)
disp(time_parts)

for i=1:length(time_loops)-1
    tab(i) = time_loops(i+1)-time_loops(i);
end

min(tab)
max(tab)

fh1 = figure(1);
fh1.WindowState = 'maximized';
hold on;
plot([1:length(tab)],tab,'LineWidth',2,'DisplayName','loop-time')
plot([1:length(time_of_ik)],time_of_ik,'LineWidth',3,'DisplayName','function compute\_joint\_positions')
legend;

tabela(:,1)=time_parts(:,1);
for j=2:7
    for i=1:length(time_parts)
        tabela(i,j) = time_parts(i,j+1)-time_parts(i,j);
    end
end

tabela=abs(tabela);

fh2 = figure(2);
fh2.WindowState = 'maximized';
hold on
plot([1:length(tab)],tab,'LineWidth',2,'DisplayName','loop-time')
% plot([1:length(time_parts)],tabela(:,1),'LineWidth',2,'DisplayName','IK-computing')
plot([1:length(time_parts)],tabela(:,2),'LineWidth',2,'Marker','o','DisplayName','time1-time2 read forces')
plot([1:length(time_parts)],tabela(:,3),'LineWidth',2,'Marker','o','DisplayName','time2-time3')
plot([1:length(time_parts)],tabela(:,4),'LineWidth',2,'Marker','o','DisplayName','time3-time4 getTransform')
plot([1:length(time_parts)],tabela(:,5),'LineWidth',2,'Marker','o','DisplayName','time4-time5')
plot([1:length(time_parts)],tabela(:,6),'LineWidth',2,'Marker','o','DisplayName','time5-time6 compute IK')
plot([1:length(time_parts)],tabela(:,7),'LineWidth',2,'Marker','o','DisplayName','time6-time7 move')
legend;


fh3 = figure(3);
fh3.WindowState = 'maximized';
plot(1:loop_count,changes_in_loop(1:loop_count),'LineWidth',2,'DisplayName','0 - not moving, 1 - moving');

% Display changes in joint positions that occured during the program run

% disp("RArm")
% 
% disp("LArm")
% 
% disp("Torso")

% draw_forces_and_movement(F_values_for_plot,all_cart_positions_L,all_cart_positions_R,array_time)

% disp(all_read_joints)
% disp(initial_joints)

%%

close(fig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS FOR LIMITS

function [stop] = inside_work_zone(cart_position)

    global x_up_limit
    global x_low_limit
    global y_up_limit
    global y_low_limit
    global z_up_limit
    global z_low_limit

    global num_limbs
    global display_frequency
    global loop_count

    for i=1:num_limbs
        if ~all(cart_position(i,1:3) < [x_up_limit,y_up_limit(i),z_up_limit])
            if mod(loop_count,display_frequency)==0
                fprintf("Upper limit of work space reached.\n")
            end
            stop = 1;
        elseif ~all(cart_position(i,1:3) > [x_low_limit,y_low_limit(i),z_low_limit])
            if mod(loop_count,display_frequency)==0
                fprintf("Lower limit of work space reached.\n")
            end
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
