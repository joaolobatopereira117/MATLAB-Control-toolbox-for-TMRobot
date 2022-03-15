function [Pose] = Forward_Kinematics_TM5_700(theta)

%---Link sizes/distances (mm):
L1 = 145.2;
L2 = 329;
L3 = 311.5;
L4 = 106;
L5 = 106;
L6 = 114.4;

%---DH parameters
DH_alpha = [0, 90, 0, 0, 90, 90]' * pi/180;
DH_a     = [0, 0, L2, L3 , 0, 0]';
DH_theta = [180, 90, 0, 90, 0, 0]' * pi/180; % Assuming all joints 0
DH_d     = [L1, 0, 0, -L4, L5, L6]';

T01_des = transf_matrix (DH_alpha(1), DH_a(1), DH_theta(1)+theta(1)*pi/180, DH_d(1));
T12_des = transf_matrix (DH_alpha(2), DH_a(2), DH_theta(2)+theta(2)*pi/180, DH_d(2));
T23_des = transf_matrix (DH_alpha(3), DH_a(3), DH_theta(3)+theta(3)*pi/180, DH_d(3));
T34_des = transf_matrix (DH_alpha(4), DH_a(4), DH_theta(4)+theta(4)*pi/180, DH_d(4));
T45_des = transf_matrix (DH_alpha(5), DH_a(5), DH_theta(5)+theta(5)*pi/180, DH_d(5));
T56_des = transf_matrix (DH_alpha(6), DH_a(6), DH_theta(6)+theta(6)*pi/180, DH_d(6));

% ShoulderP=T01_des(1:3,4);
% 
% T03=T01_des*T12_des*T23_des;
% ElbowP=T03(1:3,4);
% 
% T05=T01_des*T12_des*T23_des*T34_des*T45_des;
% WristP=T05(1:3,4);

T06=T01_des*T12_des*T23_des*T34_des*T45_des*T56_des;
     
pos=T06(1:3,4);
or=rotation2rpy(T06(1:3,1:3))*180/pi;
     
Pose(1:3)= pos;
Pose(4:6)=or;

end