
function [link_names, coordinates] = compute_cart_positions(joint_positions, joint_names, d_fk)

    %%%%%%%%%%%%%%%%%%%%
    % Forward kinematics
    %%%%%%%%%%%%%%%%%%%%

%     global d_fk;
    
    % variables for FK
    
    left_arm_joint_positions = joint_positions(1:7);
    left_arm_joint_names = joint_names(1:7);
    
    right_arm_joint_positions = joint_positions(8:14);
    right_arm_joint_names = joint_names(8:14);
    
    
    % Create client for rosservice /compute_fk
    fk_client = rossvcclient("/compute_fk","DataFormat","struct"); % create a client
    
    
    % Fill the message with necessary data
    
    %%% LEFT ARM
    
    % Create message of the correct type for this service
    fk_req_LArm = rosmessage(fk_client); % create a message
    
    fk_req_LArm.Header.FrameId = 'base_link'; % using coordinate system of the base
    fk_req_LArm.FkLinkNames = {'arm_left_1_link', 'arm_left_2_link', 'arm_left_3_link', 'arm_left_4_link', 'arm_left_5_link', 'arm_left_6_link', 'arm_left_7_link'};
    
    fk_req_LArm.RobotState.JointState.Name = left_arm_joint_names;
    fk_req_LArm.RobotState.JointState.Position = left_arm_joint_positions;
    
    %%% RIGHT ARM
    
    % Create message of the correct type for this service
    fk_req_RArm = rosmessage(fk_client);
    
    fk_req_RArm.Header.FrameId = 'base_link';
    fk_req_RArm.FkLinkNames = {'arm_right_1_link', 'arm_right_2_link', 'arm_right_3_link', 'arm_right_4_link', 'arm_right_5_link', 'arm_right_6_link', 'arm_right_7_link'};
    
    fk_req_RArm.RobotState.JointState.Name = right_arm_joint_names;
    fk_req_RArm.RobotState.JointState.Position = right_arm_joint_positions;
    
    %%% Send the message and wait for response
    
    if isServerAvailable(fk_client)
        fk_resp_LArm = call(fk_client,fk_req_LArm,"Timeout",10);
        fk_resp_RArm = call(fk_client,fk_req_RArm,"Timeout",10);
    else
        error("Service server not available on network")
    end
    
    % Check if error has occured
    
    if (fk_resp_LArm.ErrorCode.Val ~= 1) || (fk_resp_RArm.ErrorCode.Val ~= 1)
        error_name = d_fk(string(fk_resp_RArm.ErrorCode.Val));
        fprintf("FK error, value: %d, name: %s\n",fk_resp_RArm.ErrorCode.Val,error_name)
    end
    
    % Sort the data from fk response
    
    link_names = [fk_resp_LArm.FkLinkNames fk_resp_RArm.FkLinkNames];
    coordinates = [fk_resp_LArm.PoseStamped fk_resp_RArm.PoseStamped]; % link states relative to base_link!!!
