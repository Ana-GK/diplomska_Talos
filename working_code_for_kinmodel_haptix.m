% working kinmodel for MuJoCo Haptix in Windows

% r.talos_arm_haptix('URDF','Talos.urdf','InitialLink','torso_2_link','FinalLink','arm_left_7_link')

r=talos_arm_haptix;

T=[0.1;0.6;0.3];
Kp=1;
tsamp=0.01;
r.JMove(r.q_home,1)
novi_q = r.q;
e=1;
while norm(e) > 0.01
    [x,R,J]=kinmodel_talos_arm(r.q,r.TCP);
    e = T-x;
    J = J(1:3,:);
    qd=pinv(J)*Kp*e;
    novi_q= novi_q+qd*tsamp;
    r.GoTo_q(novi_q, qd*0, qd*0, 0.01);
end
