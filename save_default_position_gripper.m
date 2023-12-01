file_name = 'default_pose_grippers.xlsx';
file_path = '/home/cobotat4/Documents/MATLAB/Talos_Kuka_project/';
file = strcat(file_path, file_name);

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

x_axes=[{'left'},{'right'}];
y_axes=[{'x'},{'y'},{'z'},{'qx'},{'qy'},{'qz'},{'qw'},];

writecell(x_axes,file,'Sheet',1,'Range','B1')

writecell(y_axes',file,'Sheet',1,'Range','A2')

writematrix(initial_cart_position',file,'Sheet',1,'Range','B2')