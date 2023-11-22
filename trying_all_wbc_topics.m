

[~,msg_wbc1]=rospublisher('/whole_body_kinematic_controller/base_link_orientation/parameter_descriptions');
[~,msg_wbc2]=rospublisher('/whole_body_kinematic_controller/com_XY/com_task_XY/parameter_updates');
[~,msg_wbc3]=rospublisher('/whole_body_kinematic_controller/com_XY/com_task_XY/parameter_descriptions');
[~,msg_wbc4]=rospublisher('/whole_body_kinematic_controller/com_XY/com_task_XY/parameter_updates');
[~,msg_wbc5]=rospublisher('/whole_body_kinematic_controller/joint_limit_task/joint_limits/parameter_descriptions');
[~,msg_wbc6]=rospublisher('/whole_body_kinematic_controller/joint_limit_task/joint_limits/parameter_updates');
[~,msg_wbc7]=rospublisher('/whole_body_kinematic_controller/joint_states');
[~,msg_wbc8]=rospublisher('/whole_body_kinematic_controller/left_foot_constraint/parameter_descriptions');
[~,msg_wbc9]=rospublisher('/whole_body_kinematic_controller/left_foot_constraint/parameter_updates');
[~,msg_wbc10]=rospublisher('/whole_body_kinematic_controller/markers');
[~,msg_wbc11]=rospublisher('/whole_body_kinematic_controller/reference_task/reference_posture_arm_left_1_joint/parameter_descriptions');
[~,msg_wbc12]=rospublisher('/whole_body_kinematic_controller/reference_task/reference_posture_arm_left_1_joint/parameter_updates');
[~,msg_wbc13]=rospublisher('/whole_body_kinematic_controller/right_foot_constraint/parameter_descriptions');
[~,msg_wbc14]=rospublisher('/whole_body_kinematic_controller/right_foot_constraint/parameter_updates');
[~,msg_wbc15]=rospublisher('/whole_body_kinematic_controller/self_collision/collision_lines');
[~,msg_wbc16]=rospublisher('/whole_body_kinematic_controller/self_collision/self_collision/parameter_descriptions');
[~,msg_wbc17]=rospublisher('/whole_body_kinematic_controller/self_collision/self_collision/parameter_updates');
[~,msg_wbc18]=rospublisher('/whole_body_kinematic_controller/torso_height/parameter_descriptions');
[~,msg_wbc19]=rospublisher('/whole_body_kinematic_controller/torso_height/parameter_updates');
[~,msg_wbc20]=rospublisher('/whole_body_kinematic_controller/torso_orientation/parameter_descriptions');
[~,msg_wbc21]=rospublisher('/whole_body_kinematic_controller/torso_orientation/parameter_updates');



[~,req_wbc1]=rossvcclient('/whole_body_kinematic_controller/base_link_orientation/set_parameters');
[~,req_wbc2]=rossvcclient('/whole_body_kinematic_controller/com_XY/com_task_XY/set_parameters');
% [~,req_wbc3]=rossvcclient('/whole_body_kinematic_controller/get_stack_description');
% [~,req_wbc4]=rossvcclient('/whole_body_kinematic_controller/get_task_error');
% [~,req_wbc5]=rossvcclient('/whole_body_kinematic_controller/get_tasks_errors');
[~,req_wbc6]=rossvcclient('/whole_body_kinematic_controller/joint_limit_task/joint_limits/set_parameters');
[~,req_wbc7]=rossvcclient('/whole_body_kinematic_controller/left_foot_constraint/set_parameters');
% [~,req_wbc8]=rossvcclient('/whole_body_kinematic_controller/pop_task');
% [~,req_wbc9]=rossvcclient('/whole_body_kinematic_controller/push_task');
[~,req_wbc10]=rossvcclient('/whole_body_kinematic_controller/reference_task/reference_posture_arm_left_1_joint/set_parameters');
[~,req_wbc11]=rossvcclient('/whole_body_kinematic_controller/right_foot_constraint/set_parameters');
[~,req_wbc12]=rossvcclient('/whole_body_kinematic_controller/self_collision/self_collision/set_parameters');
[~,req_wbc13]=rossvcclient('/whole_body_kinematic_controller/torso_height/set_parameters');
[~,req_wbc14]=rossvcclient('/whole_body_kinematic_controller/torso_orientation/set_parameters');

showdetails(msg_wbc1)
showdetails(msg_wbc2)
showdetails(msg_wbc3)
showdetails(msg_wbc4)
showdetails(msg_wbc5)
showdetails(msg_wbc6)
showdetails(msg_wbc7)
showdetails(msg_wbc8)
showdetails(msg_wbc9)
showdetails(msg_wbc10)
showdetails(msg_wbc11)
showdetails(msg_wbc12)
showdetails(msg_wbc13)
showdetails(msg_wbc14)
showdetails(msg_wbc15)
showdetails(msg_wbc16)
showdetails(msg_wbc17)
showdetails(msg_wbc18)
showdetails(msg_wbc19)
showdetails(msg_wbc20)
showdetails(msg_wbc21)

showdetails(req_wbc1)
showdetails(req_wbc2)
showdetails(req_wbc6)
showdetails(req_wbc7)
showdetails(req_wbc10)
showdetails(req_wbc11)
showdetails(req_wbc12)
showdetails(req_wbc13)
showdetails(req_wbc14)