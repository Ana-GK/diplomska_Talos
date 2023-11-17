
function [LArm, RArm, times] = compute_joint_positions(new_cart_position,joint_position,d_ik,ik_client)

    %%%%%%%%%%%%%%%%%%%%
    % Inverse kinematics
    %%%%%%%%%%%%%%%%%%%%

%     global d_ik;
    
    t_init = tic;

    cart_position = new_cart_position;

%     all_joint_positions = initial_joints.Position;
    
%     LArm = all_joint_positions(1:7);
%     RArm = all_joint_positions(8:14);
%     
%     LLeg = all_joint_positions(19:24);
%     RLeg = all_joint_positions(25:30);
%     
%     Head = all_joint_positions(17:18);
%     Torso = all_joint_positions(31:32);
    
    % Read joint positions only for left arm
    
    left_arm_joint_positions = joint_position.Position(1:7);
    left_arm_joint_names = joint_position.Name(1:7);
    
    right_arm_joint_positions = joint_position.Position(8:14);
    right_arm_joint_names = joint_position.Name(8:14);

    t1 = toc(t_init);
    %%%%%%%%%%%%%%%%%%
    % Inverse kinematics
    %%%%%%%%%%%%%%%%%%
    
    % creating ik_client takes tke most time (0.16s)
%     ik_client = rossvcclient("/compute_ik",'moveit_msgs/GetPositionIK');

    t2 = toc(t_init);
    
    ik_req_LArm = rosmessage(ik_client);
    
    ik_req_LArm.IkRequest.RobotState.JointState.Position = left_arm_joint_positions;
    ik_req_LArm.IkRequest.RobotState.JointState.Name = left_arm_joint_names;
    
    ik_req_LArm.IkRequest.GroupName = 'left_arm';
    ik_req_LArm.IkRequest.IkLinkName = 'arm_left_7_link';

    ik_req_LArm.IkRequest.PoseStamped.Header.FrameId = 'base_link';
    
    
    ik_req_LArm.IkRequest.PoseStamped.Pose.Position.X = cart_position(1,1);
    ik_req_LArm.IkRequest.PoseStamped.Pose.Position.Y = cart_position(1,2);
    ik_req_LArm.IkRequest.PoseStamped.Pose.Position.Z = cart_position(1,3);
    
    ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.X = cart_position(1,4);
    ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.Y = cart_position(1,5);
    ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.Z = cart_position(1,6);
    ik_req_LArm.IkRequest.PoseStamped.Pose.Orientation.W = cart_position(1,7);

%     joint_constraint = rosmessage("moveit_msgs/JointConstraint");
%     joint_constraint.JointName = 'arm_left_7_joint';
%     joint_constraint.Position = left_arm_joint_positions(7);
%     joint_constraint.ToleranceAbove = 0.01;
%     joint_constraint.ToleranceBelow = 0.01;
%     joint_constraint.Weight = 0.5;
%     ik_req_LArm.IkRequest.Constraints.JointConstraints = joint_constraint;

%     ik_req_LArm.IkRequest.Constraints.JointConstraints.JointName = 'arm_left_7_joint';
%     ik_req_LArm.IkRequest.Constraints.JointConstraints.Position = 0.008;
    
    %%%
    
    ik_req_RArm = rosmessage(ik_client);
    
    % ik_req.IkRequest.Timeout.Sec = int32(5);
    
    ik_req_RArm.IkRequest.RobotState.JointState.Position = right_arm_joint_positions;
    ik_req_RArm.IkRequest.RobotState.JointState.Name = right_arm_joint_names;
    
    ik_req_RArm.IkRequest.GroupName = 'right_arm';
    ik_req_RArm.IkRequest.IkLinkName = 'arm_right_7_link';
    
    ik_req_RArm.IkRequest.PoseStamped.Header.FrameId = 'base_link';
    
    
    ik_req_RArm.IkRequest.PoseStamped.Pose.Position.X = cart_position(2,1);
    ik_req_RArm.IkRequest.PoseStamped.Pose.Position.Y = cart_position(2,2);
    ik_req_RArm.IkRequest.PoseStamped.Pose.Position.Z = cart_position(2,3);
    
    ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.X = cart_position(2,4);
    ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.Y = cart_position(2,5);
    ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.Z = cart_position(2,6);
    ik_req_RArm.IkRequest.PoseStamped.Pose.Orientation.W = cart_position(2,7);

    t3 = toc(t_init);
    if isServerAvailable(ik_client)
        ik_resp_LArm = call(ik_client,ik_req_LArm,"Timeout",10);
        ik_resp_RArm = call(ik_client,ik_req_RArm,"Timeout",10);
    else
        error("Service server not available on network")
    end

    t4 = toc(t_init);
    
    
    if (ik_resp_LArm.ErrorCode.Val) ~= 1 || (ik_resp_RArm.ErrorCode.Val ~= 1)
        error_name = d_ik(string(ik_resp_RArm.ErrorCode.Val));
        fprintf("FK error, value: %d, name: %s\n",ik_resp_RArm.ErrorCode.Val,error_name)
    end
    
    LArm = ik_resp_LArm.Solution.JointState.Position(15:21);
    RArm = ik_resp_RArm.Solution.JointState.Position(29:35);
    t5 = toc(t_init);
    times = [t1, t2-t1, t3-t2, t4-t3, t5-t4];
end
