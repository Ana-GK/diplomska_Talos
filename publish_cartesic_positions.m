function publish_cartesic_positions(cart_pos,pubL,msgL,pubR,msgR)
    xL=cart_pos(1,1);
    yL=cart_pos(1,2);
    zL=cart_pos(1,3);
    qxL=cart_pos(1,4);
    qyL=cart_pos(1,5);
    qzL=cart_pos(1,6);
    qwL=cart_pos(1,7);
    
    xR=cart_pos(2,1);
    yR=cart_pos(2,2);
    zR=cart_pos(2,3);
    qxR=cart_pos(2,4);
    qyR=cart_pos(2,5);
    qzR=cart_pos(2,6);
    qwR=cart_pos(2,7);

    msgL.Header.FrameId = 'odom';
    % msg.Header.FrameId = 'base_link';
    msgL.Pose.Position.X = xL;
    msgL.Pose.Position.Y = yL;
    msgL.Pose.Position.Z = zL;
    
    msgL.Pose.Orientation.X = qxL;
    msgL.Pose.Orientation.Y = qyL;
    msgL.Pose.Orientation.Z = qzL;
    msgL.Pose.Orientation.W = qwL;


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
