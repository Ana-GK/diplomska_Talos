function [] = move(LLeg,RLeg,LArm,RArm,Head,Torso,dt)

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

    dT1 = dt;

    msgP_Lleg.Positions = [LLeg]';
    msgP_Lleg.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Lleg.Points(1) = msgP_Lleg;
    msgP_Rleg.Positions = [RLeg]';
    msgP_Rleg.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Rleg.Points(1) = msgP_Rleg;
    msgP_Larm.Positions = [LArm]';
    msgP_Larm.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Larm.Points(1) = msgP_Larm;
    msgP_Rarm.Positions = [RArm]';
    msgP_Rarm.TimeFromStart = ros.msg.Duration(dT1);   
    msg_Rarm.Points(1) = msgP_Rarm;
    msgP_head.Positions = [Head]';
    msgP_head.TimeFromStart = ros.msg.Duration(dT1);   
    msg_head.Points(1) = msgP_head;
    msgP_torso.Positions = [Torso]';
    msgP_torso.TimeFromStart = ros.msg.Duration(dT1);   
    msg_torso.Points(1) = msgP_torso;
    
    send(pub_Lleg,msg_Lleg);
    send(pub_Rleg,msg_Rleg);
    send(pub_Larm,msg_Larm);
    send(pub_Rarm,msg_Rarm);
    send(pub_head,msg_head);
    send(pub_torso,msg_torso);
end