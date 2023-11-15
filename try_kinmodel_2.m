
clear all;

rosshutdown;
rosinit;

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

global sub_states;
global msg_states;

sub_states = rossubscriber('/joint_states');
msg_states = receive(sub_states, 1); % 1 is timeout in seconds

initial_joint_positions = msg_states.Position; % Save initial joint positions
initial_joint_names = msg_states.Name; % Save joint names

LArm = initial_joint_positions(1:7);
RArm = initial_joint_positions(8:14);

LLeg = initial_joint_positions(19:24);
RLeg = initial_joint_positions(25:30);

Head = initial_joint_positions(17:18);
Torso = initial_joint_positions(31:32);

[p,R,J]=kinmodel_talos_left_arm(LArm);

p_new(1) = p(1);
p_new(2) = p(2);
p_new(3) = p(3)+0.063;

disp(p_new)

T=[0.21;0.3047;-0.0115];
Kp=1;
tsamp=1;
dt=2;
move(LLeg,RLeg,LArm,RArm,Head,Torso,dt)
% q = [LArm' RArm' LLeg' RLeg' Head' Torso']'
q=initial_joint_positions;
novi_q = LArm;
e=1;
while norm(e) > 0.02
    [x,R,J]=kinmodel_talos_left_arm(q);
    e = T-x;
    J = J(1:3,:);
    qd=pinv(J)*Kp*e;
    novi_q= novi_q+qd*tsamp;
    move(LLeg,RLeg,novi_q(1:7),RArm,Head,Torso,10);
    disp("Here")
    disp(norm(e))
%     pause(0.01)
end

% omejitev premika!
