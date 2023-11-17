
% Useful

open('generate_kinmodel_corrected_script.m')
open('Talos.xml')
open('Panda.xml')
open('Panda_Chain.xml')
open('Talos_Kuka.xml')
open('ati_ft.m')
open('kinmodel_talos_left_arm.m')

% % % % % % % % % % % % lwr_FRI % za realnega robota
% % % % % % % % % % % % kinmodel_lwr % kinematični model

[pub_wbc,msg_wbc]=rospublisher('/whole_body_kinematic_controller/com_XY/com_task_XY/parameter_updates');

% % % % % % % % % % % % r.talos_arm_haptix('URDF','Talos.urdf','InitialLink','torso_2_link','FinalLink','arm_left_7_link')