

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

% [d_fk,d_ik] = error_codes();
% 
% sub_states = rossubscriber('/joint_states');
% msg_states = receive(sub_states, 5); % 5 is timeout in seconds
% 
% joint_positions = msg_states.Position; % Save initial joint positions
% joint_names = msg_states.Name; % Save joint names
% 
% [~, coordinates] = compute_cart_positions_wbc(joint_positions, joint_names, d_fk);
% 
% for i=1:2
%     initial_cart_position(i,1) = coordinates(7,i).Pose.Position.X;
%     initial_cart_position(i,2) = coordinates(7,i).Pose.Position.Y;
%     initial_cart_position(i,3) = coordinates(7,i).Pose.Position.Z;
%     initial_cart_position(i,4) = coordinates(7,i).Pose.Orientation.X;
%     initial_cart_position(i,5) = coordinates(7,i).Pose.Orientation.Y;
%     initial_cart_position(i,6) = coordinates(7,i).Pose.Orientation.Z;
%     initial_cart_position(i,7) = coordinates(7,i).Pose.Orientation.W;
% end

xL=initial_cart_position(1,1);
yL=initial_cart_position(1,2);
zL=initial_cart_position(1,3);
qxL=initial_cart_position(1,4);
qyL=initial_cart_position(1,5);
qzL=initial_cart_position(1,6);
qwL=initial_cart_position(1,7);

xR=initial_cart_position(2,1);
yR=initial_cart_position(2,2);
zR=initial_cart_position(2,3);
qxR=initial_cart_position(2,4);
qyR=initial_cart_position(2,5);
qzR=initial_cart_position(2,6);
qwR=initial_cart_position(2,7);



zL=zL-0.15;
zR=zR-0.15;

disp(zL)
disp(zR)
%%

[pubL,msgL]=rospublisher('/whole_body_kinematic_controller/gripper_left','geometry_msgs/PoseStamped');

msgL.Header.FrameId = 'odom';
% msg.Header.FrameId = 'base_link';
msgL.Pose.Position.X = xL;
msgL.Pose.Position.Y = yL;
msgL.Pose.Position.Z = zL;

msgL.Pose.Orientation.X = qxL;
msgL.Pose.Orientation.Y = qyL;
msgL.Pose.Orientation.Z = qzL;
msgL.Pose.Orientation.W = qwL;

[pubR,msgR]=rospublisher('/whole_body_kinematic_controller/gripper_right','geometry_msgs/PoseStamped');

msgR.Header.FrameId = 'odom';
% msg.Header.FrameId = 'base_link';
msgR.Pose.Position.X = xR;
msgR.Pose.Position.Y = yR;
msgR.Pose.Position.Z = zR;

msgR.Pose.Orientation.X = qxR;
msgR.Pose.Orientation.Y = qyR;
msgR.Pose.Orientation.Z = qzR;
msgR.Pose.Orientation.W = qwR;

send(pubL,msgL);
send(pubR,msgR);

% test if the topic works
sub=rossubscriber('/whole_body_kinematic_controller/gripper_left');
left = sub.LatestMessage
sub=rossubscriber('/whole_body_kinematic_controller/gripper_right');
right = sub.LatestMessage
