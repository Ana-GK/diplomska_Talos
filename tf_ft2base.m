function [F] = tf_ft2base(quat,force)
    % transforms the force from frame of force sensor to frame of base link

    % quat is a vector of quaternions:
    % > quat = [w x y z] !!! order is important
    % force is a vector of force components from ft sensor:
    % > force = [Fx Fy Fz]

[a,b] = size(force);

% transponse force if necessary
if a == 3
    % force = force;
elseif b == 3
    force = force';
else
    disp('Incorrect dimensions of force vector.')
end

% find rotation matrix
R = q2r(quat);

% multiply R with new vector of force
F = R*force;