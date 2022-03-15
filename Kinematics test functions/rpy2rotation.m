function [R_rpy] = rpy2rotation (yaw, pitch, roll)
%input:
%yaw (theta) -> rotation in x
%pitch (psi) -> rotation in y
%roll (phi)  -> rotation in z

%return: 
%3x3 rotation matrix

theta = yaw;
psi = pitch;
phi = roll;

R_rpy = [cos(phi)*cos(psi) -sin(phi)*cos(theta)+cos(phi)*sin(psi)*sin(theta) sin(phi)*sin(theta)+cos(phi)*sin(psi)*cos(theta);
         sin(phi)*cos(psi) cos(phi)*cos(theta)+sin(phi)*sin(psi)*sin(theta)  -cos(phi)*sin(theta)+sin(phi)*sin(psi)*cos(theta);
         -sin(psi)         cos(psi)*sin(theta)                                cos(psi)*cos(theta)                              ];
end
