function [theta, psi, phi] = rotation2rpy(rot)

%input: 
%3x3 rotation matrix

%return:
%yaw (theta) -> rotation in x
%pitch (psi) -> rotation in y
%roll (phi)  -> rotation in z

psi = atan2 (-rot(3,1), sqrt(rot(1,1)^2 + rot(2,1)^2));

if abs(psi - pi/2) < 0.01
    phi = 0;
    theta = atan2(rot(1,2), rot(2,2));
elseif abs(psi + pi/2) < 0.01
    phi = 0;
    theta = -atan2(rot(1,2), rot(2,2));
else
     phi = atan2(rot(2,1)/cos(psi), rot(1,1)/cos(psi));
     theta = atan2(rot(3,2)/cos(psi), rot(3,3)/cos(psi));

end