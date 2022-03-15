    function [teta] = Inverse_Kinematics_TM5_700(Pose_Tip_des)
    
    %---Link sizes/distances (mm):
    L1 = 145.2;
    L2 = 329;
    L3 = 311.5;
    L4 = -122.2;
    L5 = 106;
    L6 = 114.4;
    
    %---DH parameters
    DH_alpha = [0, -90, 0, 0, 90, 90]' * pi/180;
    DH_a     = [0, 0, L2, L3 , 0, 0]';
    DH_theta = [0, -90, 0, 90, 0, 0]' * pi/180; % Assuming all joints 0
    DH_d     = [L1, 0, 0, L4, L5, L6]';
    %                                  InverseKinematics.m
    %--------------------------------------------------------------------------------------------
    %-----------------INPUT: Desired tip pose (x,y,z,yaw_x_deg,pitch_y_deg,roll_z_deg)----------- 
    
    pe0_des                 = Pose_Tip_des(1:3); % End effector XYZ coordinates
    Desired_Tip_orientation = Pose_Tip_des(4:6)*pi/180; % End effector Orientation
    
    %---Compute desired R06: orientation of Tip (6) with respect to Base (0) from RPY angles
    yaw_x   = Desired_Tip_orientation(1);
    pitch_y = Desired_Tip_orientation(2);
    roll_z  = Desired_Tip_orientation(3);
    R06_des = rpy2rotation(yaw_x , pitch_y , roll_z);  % Desired Rotation matrix
    
    %---Compute desired matrix T06_des
    T06_des         = eye(4,4); % Identity matrix 4x4 will be filled with Desired 
                                % Rotation and Desired Position
    T06_des(1:3,1:3)= R06_des;  
    T06_des(1:3,4)  = pe0_des;  
    
    %---Compute teta1
    P05    = T06_des*[0 0 -L6 1]'; % Position of Wrist 2 (5) with respect to Base (0)
    P05_x  = P05(1,1);
    P05_y  = P05(2,1);
    P05_xy = sqrt(P05_x^2 + P05_y^2);
    
    alpha1 = atan2(P05_y,P05_x); % Angle between x0 and 0P5_xy
    
    
    alpha2 = - acos(L4/abs(P05_xy));
    
    
    teta1     = alpha1 + alpha2 + pi/2;
    teta1_deg = teta1 * 180/pi; % Conversion from rad to degrees
    
    
    %---Compute teta5
    P06_x = T06_des(1,4);
    P06_y = T06_des(2,4);
    
    
    teta5 = acos((P06_x*sin(teta1)-P06_y*cos(teta1)-L4)/L6);
        
    
    teta5_deg = teta5 * 180/pi; % Conversion from rad to degrees
    
    %---Compute teta6
    T60   = (T06_des)^(-1);  % Position of Base (0) with respect to Tip (6)
    X60_x = T60(1,1);
    X60_y = T60(2,1);
    Y60_x = T60(1,2);
    Y60_y = T60(2,2);
    
    % Conversion of 6Y1 from spherical to cartesian coordinates
    Y61 = [-X60_x*sin(teta1)+Y60_x*cos(teta1);
           -X60_y*sin(teta1)+Y60_y*cos(teta1)]; % Y orientation of Shoulder (1) with respect to Tip (6)
    
    teta6     = atan2(Y61(2,1)/sin(teta5),-Y61(1,1)/sin(teta5));
    teta6_deg = teta6 * 180/pi;
    
    
    %---Compute teta3
    
    % Calculation of T_01, T_45 and T_56, since teta1, teta 4 and teta6 are now known
    T_01 = transf_matrix(DH_alpha(1), DH_a(1),DH_theta(1)+teta1, DH_d(1));
    T_45 = transf_matrix(DH_alpha(5), DH_a(5),DH_theta(5)+teta5, DH_d(5));
    T_56 = transf_matrix(DH_alpha(6), DH_a(6),DH_theta(6)+teta6, DH_d(6));
    
    T_15 = (T_01)^(-1)*T06_des*(T_56)^(-1); % Position of Shoulder (1) with respect to Wrist 2 (5)
    T_14 = T_15*(T_45)^(-1); % Position of Shoulder (1) with respect to Wrist 1 (4)
    
    P14_x  = T_14(1,4);
    P14_z  = T_14(3,4);
    P14_xz = sqrt(P14_x^2+P14_z^2); % Distance from Shoulder to Wrist 1
    
    
    teta3 = - acos(((P14_xz^2)-(L2^2)-(L3^2))/(2*L2*L3));
    
    
    teta3_deg = teta3 * 180/pi; % Conversion from rad to degrees
    
    %---Compute teta2
    phi_3 = pi-teta3; % Since teta3 = -cos(phi3)
    
    % 3 DOF method
    phi_1 = atan2(-P14_z,P14_x); 
    phi_2 = asin((-L3*sin(phi_3))/abs(P14_xz));
    
    teta2     = - (phi_1 - phi_2);
    teta2_deg = teta2 * 180/pi; % Conversion from rad to degrees
    
    %---Compute teta4
    
    % Calculation of T_12 and T_23, since teta2, teta3 are now known
    T_12 = transf_matrix(DH_alpha(2), DH_a(2),DH_theta(2)+teta2, DH_d(2));
    T_23 = transf_matrix(DH_alpha(3), DH_a(3),DH_theta(3)+teta3, DH_d(3));
    
    T_24 = (T_12)^(-1)*T_14; % Position of Wrist (4) with respect to Shoulder (2)
    T_34 = (T_23)^(-1)*T_24; % Position of Wrist (4) with respect to Elbow (3)
    
    X34_x = T_34(1,1);
    X34_y = T_34(2,1);
    
    teta4     = atan2(X34_y,X34_x);
    teta4_deg = teta4 * 180/pi; % Conversion from rad to degrees
    
    teta = [teta1_deg teta2_deg teta3_deg teta4_deg teta5_deg teta6_deg];
    
    end