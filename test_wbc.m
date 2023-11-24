
[d_fk,d_ik] = error_codes();

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 5); % 5 is timeout in seconds

joint_positions = msg_states.Position; % Save initial joint positions
joint_names = msg_states.Name; % Save joint names

[~, coordinates] = compute_cart_positions_wbc(joint_positions, joint_names, d_fk);

for i=1:2
    initial_cart_position(i,1) = coordinates(7,i).Pose.Position.X;
    initial_cart_position(i,2) = coordinates(7,i).Pose.Position.Y;
    initial_cart_position(i,3) = coordinates(7,i).Pose.Position.Z;
    initial_cart_position(i,4) = coordinates(7,i).Pose.Orientation.X;
    initial_cart_position(i,5) = coordinates(7,i).Pose.Orientation.Y;
    initial_cart_position(i,6) = coordinates(7,i).Pose.Orientation.Z;
    initial_cart_position(i,7) = coordinates(7,i).Pose.Orientation.W;
end

x=initial_cart_position(2,1);
y=initial_cart_position(2,2);
z=initial_cart_position(2,3);
qx=initial_cart_position(2,4);
qy=initial_cart_position(2,5);
qz=initial_cart_position(2,6);
qw=initial_cart_position(2,7);

disp(x)
disp(y)
disp(z)
disp(qx)
disp(qy)
disp(qz)
disp(qw)

z=z+0.1;

disp(z)

[pub,msg]=rospublisher('/whole_body_kinematic_controller/reference_ref','geometry_msgs/PoseStamped');

msg.Header.FrameId = 'odom';
msg.Pose.Position.X = x;
msg.Pose.Position.Y = y;
msg.Pose.Position.Z = z;

msg.Pose.Orientation.X = qx;
msg.Pose.Orientation.Y = qy;
msg.Pose.Orientation.Z = qz;
msg.Pose.Orientation.W = qw;


send(pub,msg);

% test if the topic works
sub=rossubscriber('/whole_body_kinematic_controller/reference_ref');
sub.LatestMessage
