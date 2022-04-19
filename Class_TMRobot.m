%This class provides a set of methods that allows u to control and monitor an omron TMRobot thru modbus protocol and Listen Node Socket Tcp listenner
%It has been created around the TM5_700 cobot so the default constructor will create a virtual model of this robot as the IP adress and ports that i'have assigned
%U can choose not to use this class and control TMrobot using the individual functions as the pick and place example shows     
%To support the simulation portion of this project it is used the PETER Corke Robotics Toolbox (RTB file) 

classdef Class_TMRobot

    properties
        sym_model
        iprobot
        socketport
        modbusport
    end

    properties(SetAccess = private)
        modbusClient
        tcpClient
    end

    methods
    
        function r=Class_TMRobot(varargin)
            

            addpath(genpath('Individual function Library'));

            L(1)= Revolute('d', 145.1, ...   % link length (Dennavit-Hartenberg notation)
                'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', -pi/2, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-277 277]*pi/180 ); % minimum and maximum joint angle
            
            L(2)= Revolute('offset', -pi/2, ...%-pi/2
                'd', 0, ...   % link length (Dennavit-Hartenberg notation)
                'a', 329, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle
            
            L(3)= Revolute('offset', 0, ...
                'd', 0, ...   % link length (Dennavit-Hartenberg notation)
                'a', 311.5, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-162 162]*pi/180 ); % minimum and maximum joint angle
            
            L(4)= Revolute('offset', pi/2, ...%pi/2
                'd', -122.2, ...   % link length (Dennavit-Hartenberg notation)
                'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle
            
            L(5)= Revolute('offset', 0, ...
                'd', 106, ...   % link length (Dennavit-Hartenberg notation)
                'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-187 187]*pi/180 ); % minimum and maximum joint angle
            
            L(6)= Revolute('offset', 0, ...
                'd', 114.4, ...   % link length (Dennavit-Hartenberg notation)
                'a', 0, ...               % link offset (Dennavit-Hartenberg notation)
                'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
                'qlim', [-277 277]*pi/180 ); % minimum and maximum joint angle

            default.sym_model=SerialLink(L,'name', 'TM5_700'); %Default options for the tm5_700 robot
            default.iprobot="192.168.0.111";
            default.socketport=5890;
            default.modbusport=502;
            
            opt.sym_model = [];
            opt.iprobot = [];
            opt.socketport=[];
            opt.modbusport=[];

            [opt]=tb_optparse(opt, varargin);

            for p=properties(r)'
                p = p{1};  % property name
                
                if isfield(opt, p) && ~isempty( opt.(p) )
                    % if there's a set option, override what's in the robot
                    r.(p) = opt.(p);
                end
                
                if isfield(default, p) && isempty( r.(p) ) 
                    % otherwise if there's a set default, use that
                    r.(p) = default.(p);
                end
             end

            

        end

%% Connection functions        

        function [r]=connect2controller_c(r)
            
            [r.tcpClient,r.modbusClient]=connect2controller(r.iprobot,r.socketport,r.modbusport);

        end

        function [r]=disconnect_c(r)

            r.modbusClient=[];
            r.tcpClient=[];

        end

%% Kinematics functions

        function [Pose] = F_Kinematics_c(r,q)
            
            Pose=F_Kinematics(r.sym_model,q);
                
        end

        function [q] = I_Kinematics_c(r,Pose)

            q=I_Kinematics(r.sym_model,Pose);
                
        end

%% Get/Set functions 

        function [q]=getRealjoints_c(r)
            q=getRealjoints(r.modbusClient);
        end
        

        function [pos]=getRealpos_c(r)

            pos=getRealpos(r.modbusClient);         
        end
        

        function [currBase]=getBase_c(r)

            currBase=getBase(r.modbusClient);

        end

        function [runT]=RunTimProj_c(r)

           runT=RunTimProj(r.modbusClient);

        end

        function [state]=getDI_c(r,n)

            state=getDI(r.modbusClient,n);

        end

        function [state]=getDO_c(r,n)

            state=getDO(r.modbusClient,n);

        end

        function setDO_c(r,n,value)

            setDO(r.modbusClient,n,value);

        end

        function setHeadlight_c(r,state)

            setHeadlight(r.modbusClient,state);

        end

        function [check]=setBase_c(r,newbase)

             check=setBase(r.tcpClient,newbase);
     
        end

%% Movement functions
        
        function [check,move]=circleMove_c(r,Endpose,Passpose,arcAng,velperc)

                q=r.isposReach_c(Passpose)+r.isposReach_c(Endpose);

                if(q==2)
                
                [check,move]=circleMove(r.tcpClient,r.modbusClient,Endpose,Passpose,arcAng,velperc);
            
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end
        
        end


        function [check,move]=LineMove2pos_c(r,pose,velperc)

                q=r.isposReach_c(pose);

                if (q==1)
            
                    [check,move]=LineMove2pos(r.tcpClient,pose,velperc);
                   
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end

        end

        function [check,move]=PTPMove2pos_c(r,pose,velperc)

                q=r.isposReach_c(pose);
            
                if(q==1)
            
                   [check,move]=PTPMove2pos(r.tcpClient,pose,velperc);
            
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end
        end

        function [check,move]=Move2joint_c(r,pose,velperc)

                q=r.isjointReach_c(pose);
                
                if(q==1)
            
                    [check,move]=Move2joint(r.tcpClient,pose,velperc);
            
                else
                    disp("Unreachable pose");
                    move=0;
                    check=0;
                end
        end

        function [check,move]=MoveJ1_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(1)=currentjointpose(1)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ2_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(2)=currentjointpose(2)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ3_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(3)=currentjointpose(3)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ4_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(4)=currentjointpose(4)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ5_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(5)=currentjointpose(5)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ6_c(r,value,velperc)

            currentjointpose=r.getRealjoints_c();
        
            currentjointpose(6)=currentjointpose(6)+value;
        
            [check,move]=r.Move2joint_c(currentjointpose,velperc);
            
        end

        function [check,move]=MoveX_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(1)=currentcartesianpose(1)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveY_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(2)=currentcartesianpose(2)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveZ_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(3)=currentcartesianpose(3)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRx_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(4)=currentcartesianpose(4)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRy_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(5)=currentcartesianpose(5)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRz_c(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos_c();
        
            currentcartesianpose(6)=currentcartesianpose(6)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos_c(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos_c(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

%% Sym functions

        function symControl_c(r)
            
            if(isobject(r.modbusClient))
                q=r.getRealjoints_c();
                
                symControl(r.sym_model,q);

            else
                symControl(r.sym_model);
            end

        end

        function [q]=getSymjoints_c(r)      

            q=getSymjoints(r.sym_model);

        end

        function [pos]=getSympos_c(r) 

            q=r.getSymjoints_c();
            pos=r.F_Kinematics_c(q);

        end

        function symJoint_c(r,Pose)

            if(nargin==1)

                q=r.getRealjoints_c();
                
                symJoint(r.sym_model,q);
            else
                symJoint(r.sym_model,Pose);
            end            
        end
         

%% Gripper functions

        function opengripper_c(r)

           opengripper(r.modbusClient,r.tcpClient);
        
        end

        function closegripper_c(r)
    
            closegripper(r.modbusClient,r.tcpClient);
        
        end

%% Test arm configuration functions

        function [check]= isjointReach_c(r,q)

           [check]=isjointReach(r.sym_model,q);

        end

        function [check]=isposReach_c(r,pos)
                
            [check]=isposReach(r.sym_model,pos);

        end

    end

end


