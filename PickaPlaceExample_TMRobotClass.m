%THIS example utilizes the TMRobot Class to develop a pick and place
%task, with the placement of three connectors on a PCB board; 
%When the board is full we press a button connected to DI 7 to restart
%the process;

%% Connection
TM5_700=Class_TMRobot();
TM5_700=TM5_700.connect2controller_c();%establish connection

%% Poses
home_pos    = [220.004 -200.002 350.004 180 -0.001 141.999];
component   = [160.501 -411.200 252.001 179.990 0.005 141.448];
pcb         = [454.298 135.495 224.457 179.891 0.04 139.8];

%% Movement
TM5_700.closegripper_c();

while(1)
    for i=0:2
            
        TM5_700.LineMove2pos_c(home_pos,100);   % initial position ; move with 100% velocity
        disp('Home Position Reached');
        
        TM5_700.LineMove2pos_c([(component(1)+i*0.613) (component(2)+i*55.468) component(3) component(4) component(5) component(6)],100);  % 1st component position ; move with 100% velocity
        disp('Ready to pick component');
        TM5_700.MoveZ_c(-30,"Line",50);                      % movement in Z axis to pick the component ; 50% velocity
        disp('Grabbing component...')
        TM5_700.opengripper_c();                              % open gripper to grab component
        
        TM5_700.MoveZ_c(50,"Line",50);                       % movement in Z axis to get away from the board ; 50% velocity
        disp('Component acquired');
        TM5_700.PTPMove2pos_c(home_pos,100);   
        disp('Moving component to PCB place position');
        TM5_700.LineMove2pos_c([(pcb(1)-i*51.5) (pcb(2)+i*0.752) pcb(3) pcb(4) pcb(5) pcb(6)] ,100);        % 1st position to place component in PCB; move with 100% velocity
        disp('Ready for placement');
        TM5_700.MoveZ_c(-30,"Line",10);                      % movement in Z axis to get close to PCB ; 50% velocity
        
        TM5_700.closegripper_c();                             % close gripper to release component
        disp('Component inserted')
        TM5_700.MoveZ_c(30,"Line",50);                       % movement in Z axis to get away from the PCB ; 50% velocity
    end

    TM5_700.LineMove2pos_c(home_pos,100);

    while(TM5_700.getDI_c(7)==0) %waiting for button pressed;
    end

end   