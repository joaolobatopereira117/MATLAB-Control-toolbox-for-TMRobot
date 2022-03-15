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

        function [r,check]=connect2controller(r)
            
            r.modbusClient=[];
            
            r.tcpClient=[];


            r.modbusClient=modbus("tcpip",r.iprobot,r.modbusport);

            r.tcpClient=tcpclient(r.iprobot,r.socketport);
        
            r.tcpClient.configureTerminator("CR/LF");
        
            flush(r.tcpClient);
        
            writeline(r.tcpClient,"$TMSTA,2,00,*41");
        
            response=char(readline(r.tcpClient));
        
            if(response(14:17)=="true")
        
                check=1;
                disp("Connected to Modbus server and Listen Node in External script control");
        
            elseif(response(13:17)=="false")
                check=0;
                disp("Connection failed!!!");
            end

        end

        function [r]=disconnect(r)

            r.modbusClient=[];
            r.tcpClient=[];

        end

%% Kinematic functions

        function [Pose] = F_Kinematics(r,q)
            
            posMatrix=r.sym_model.fkine(q*pi/180);

            Pose(1:3)=posMatrix.t';
            
            rotMatrix=[posMatrix.n posMatrix.o posMatrix.a];

            [Rx,Ry,Rz]=rotation2rpy(rotMatrix);
            
            Pose(4)=Rx*180/pi;
            Pose(5)=Ry*180/pi;
            Pose(6)=Rz*180/pi;
                
        end

        function [q] = I_Kinematics(r,Pose)

            posMatrix=transl(Pose(1),Pose(2),Pose(3))*trotx(Pose(4)*pi/180)*troty(Pose(5)*pi/180)*trotz(Pose(6)*pi/180);

            j=r.sym_model.ikcon(posMatrix);

            q=j*180/pi;
                
        end

%% Get/Set functions 

        function [q]=getRealjoints(r)

            for i=1:6

                q(i)=read(r.modbusClient,"inputregs",(7012+i*2),1,1,"single");

            end            
        end
        
        function [q]=getSymjoints(r)      

            q=r.sym_model.getpos()*180/pi;

        end

        function [pos]=getRealpos(r)

            for i=1:6

                pos(i)=read(r.modbusClient,"inputregs",(7000+i*2),1,1,"single");

            end            
        end
        
        function [pos]=getSympos(r) 

            q=r.getSymjoints();
            pos=r.F_Kinematics(q);

        end

        function [currBase]=getBase(r)

            for i=1:6
                currBase(i)=read(r.modbusClient,"inputregs",(8299+i*2),1,1,"single");
            end  

        end

        function [runT]=RunTimProj(r)

           q=1;
           for i=1:5
                if(i~=2)
                    runT(q)=read(r.modbusClient,"inputregs",8200+i,1);
                    q=q+1;
                end            
            end

            display=['Project running for ' num2str(runT(1)) ' days ' num2str(runT(2)) ' hours ' num2str(runT(3)) ' minutes and ' num2str(runT(4))  ' seconds'];

            disp(display);
        end

        function [state]=getDI(r,n)
            if(n>=0)&&(n<=15)
                state=read(r.modbusClient,"inputs",n+1,1);
            else
                disp("Number of input incorrect");
                state=3;
            end
        end

        function [state]=getDO(r,n)

            if(n>=0)&&(n<=14)
                state=read(r.modbusClient,"coils",n+1,1);
            else
                disp("Number of output incorrect");
                state=3;
            end

        end

        function setDO(r,n,value)

             if(n>=0)&&(n<=14)
                write(r.modbusClient,"coils",n+1,value);
             else
                disp("Number of output incorrect");
             end

        end

        function setHeadlight(r,state)
            write(r.modbusClient,"coils",7207,state);
        end

        function [check]=setBase(r,newbase)

                flush(r.tcpClient);
        
                data=['1,ChangeBase(' num2str(newbase(1)) ',' num2str(newbase(2)) ',' num2str(newbase(3)) ',' num2str(newbase(4)) ',' num2str(newbase(5)) ',' num2str(newbase(6)) ')'];
        
                headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
            
                fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
           
                writeline(r.tcpClient,fullcomand);
        
                message=char(readline(r.tcpClient));

                if(message(12:13) == "OK")
                    check=1;
                    disp('Base changed');
                else
                    check=0;
                    disp('Base change error');
                end
     
        end

%% Movement functions
        
        function [check,move]=circleMove(r,Endpose,Passpose,arcAng,velperc)
            
                flush(r.tcpClient);
            
            %check if reachable poses

                q=r.isposReach(Passpose)+r.isposReach(Endpose);
                
            %Verifying if Circle Trajectorie is possible
            
                Initpose=r.getRealpos();
            
                c=0;
                x=0;
                y=0;
                z=0;
            
                for i=1:6
                    if(Initpose(i)==Endpose(i) || Initpose(i)==Passpose(i))
                        c=c+1;
                    end
                end
                
                if(Initpose(1)==Endpose(1) && Initpose(1)==Passpose(1))
                    x=1;
                end
            
                if(Initpose(2)==Endpose(2) && Initpose(2)==Passpose(2))
                    y=1;
                end
            
                if(Initpose(3)==Endpose(3) && Initpose(3)==Passpose(3))
                    z=1;
                end
            
                if(c==6 || (x+y+z)>=2)
                    q=0;
                end
            
                if(q==2)
            
                    data=['1,Circle("CPP",' num2str(Passpose(1)) ',' num2str(Passpose(2)) ',' num2str(Passpose(3)) ',' num2str(Passpose(4)) ',' num2str(Passpose(5)) ',' num2str(Passpose(6)) ',' num2str(Endpose(1)) ',' num2str(Endpose(2)) ',' num2str(Endpose(3)) ',' num2str(Endpose(4)) ',' num2str(Endpose(5)) ',' num2str(Endpose(6)) ',' num2str(velperc) ',100,0,' num2str(arcAng) ' ,false)' char(13) newline 'QueueTag(1)'];
            
                    headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
                
                    fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
               
                    writeline(r.tcpClient,fullcomand);
            
                    message=char(readline(r.tcpClient));
            
                    if(message(12:13) == "OK")
                        check='Valid_comand';
                    else
                        check='Invalid_comand';
                    end
            
                    while(r.tcpClient.NumBytesAvailable==0)
                    end
            
                    message=char(readline(r.tcpClient));
            
                    if(message(17:20)=="true")
                        move='Movement executed';
            
                    else
                        move='Movement not completed';
                    end
            
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end
        
        end


        function [check,move]=LineMove2pos(r,pose,velperc)

                flush(r.tcpClient);
            
                q=r.isposReach(pose);

                if (q==1)
            
                    data=['1,Line("CPP",' num2str(pose(1)) ',' num2str(pose(2)) ',' num2str(pose(3)) ',' num2str(pose(4)) ',' num2str(pose(5)) ',' num2str(pose(6)) ',' num2str(velperc) ',100,0,false)' char(13) newline 'QueueTag(1)'];
            
                    headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
                
                    fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
               
                    writeline(r.tcpClient,fullcomand);
            
                    message=char(readline(r.tcpClient));
            
                    if(message(12:13) == "OK")
                        check='Valid_comand';
                    else
                        check='Invalid_comand';
                        move='Movement not completed';
                        return
                    end
            
                    while(r.tcpClient.NumBytesAvailable==0)
                    end
            
                    message=char(readline(r.tcpClient));
            
                    if(message(17:20)=="true")
                        move='Movement executed';
            
                    else
                        move='Movement not completed';
                    end
                   
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end

        end

        function [check,move]=PTPMove2pos(r,pose,velperc)

                flush(r.tcpClient);

                q=r.isposReach(pose);
            
                if(q==1)
            
                    data=['1,PTP("CPP",' num2str(pose(1)) ',' num2str(pose(2)) ',' num2str(pose(3)) ',' num2str(pose(4)) ',' num2str(pose(5)) ',' num2str(pose(6)) ',' num2str(velperc) ',100,0,false)' char(13) newline 'QueueTag(1)'];
            
                    headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
                
                    fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
               
                    writeline(r.tcpClient,fullcomand);
            
                    message=char(readline(r.tcpClient));
            
                    if(message(12:13) == "OK")
                        check='Valid_comand';
                    else
                        check='Invalid_comand';
                        move='Movement not completed';
                        return;
                    end
            
                    while(r.tcpClient.NumBytesAvailable==0)
                    end
            
                    message=char(readline(r.tcpClient));
            
                    if(message(17:20)=="true")
                        move='Movement executed';
            
                    else
                        move='Movement not completed';
                    end
            
                else
                    disp("Unreachable pose");
                    check=0;
                    move=0;
                end
        end

        function [check,move]=Move2joint(r,pose,velperc)

                flush(r.tcpClient);
                
                q=r.isjointReach(pose);
                
                if(q==1)
            
                    data=['1,PTP("JPP",' num2str(pose(1)) ',' num2str(pose(2)) ',' num2str(pose(3)) ',' num2str(pose(4)) ',' num2str(pose(5)) ',' num2str(pose(6)) ',' num2str(velperc) ',100,0,false)' char(13) newline 'QueueTag(1)'];
            
                    headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
                
                    fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
               
                    writeline(r.tcpClient,fullcomand);
            
                    message=char(readline(r.tcpClient));
            
                    if(message(12:13) == "OK")
                        check='Valid_comand';
                    else
                        check='Invalid_comand';
                        move='Movement not completed';
                        return;
                    end
            
                    while(r.tcpClient.NumBytesAvailable==0)
                    end
            
                    message=char(readline(r.tcpClient));
            
                    if(message(17:20)=="true")
                        move='Movement executed';
            
                    else
                        move='Movement not completed';
                    end
            
                else
                    disp("Unreachable pose");
                    move=0;
                    check=0;
                end
        end

        function [check,move]=MoveJ1(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(1)=currentjointpose(1)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ2(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(2)=currentjointpose(2)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ3(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(3)=currentjointpose(3)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ4(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(4)=currentjointpose(4)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ5(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(5)=currentjointpose(5)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveJ6(r,value,velperc)

            currentjointpose=r.getRealjoints();
        
            currentjointpose(6)=currentjointpose(6)+value;
        
            [check,move]=r.Move2joint(currentjointpose,velperc);
            
        end

        function [check,move]=MoveX(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(1)=currentcartesianpose(1)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveY(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(2)=currentcartesianpose(2)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveZ(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(3)=currentcartesianpose(3)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRx(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(4)=currentcartesianpose(4)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRy(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(5)=currentcartesianpose(5)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

        function [check,move]=MoveRz(r,value,config ,velperc)

            currentcartesianpose=r.getRealpos();
        
            currentcartesianpose(6)=currentcartesianpose(6)+value;
        
            if(config=="Line")
    
                [check,move]=r.LineMove2pos(currentcartesianpose,velperc);
            
            elseif(config=="PTP")
    
                [check,move]=r.PTPMove2pos(currentcartesianpose,velperc);
    
            else
                check=("Error on Motion config");
                move=0;
            end           
        end

%% Sym functions
        
        function symPos(r,Pose) %Fix

            if(nargin==1)

                q=r.getRealjoints();
                
                r.sym_model.plot(q*pi/180);
            else

                q=r.I_Kinematics(Pose);
                r.sym_model.plot(q*pi/180);
            end            
        end
        
        function symJoint(r,Pose)

            if(nargin==1)

                q=r.getRealjoints();
                
                r.sym_model.plot(q*pi/180);
            else

                r.sym_model.plot(Pose*pi/180);
            end            
        end

        function symControl(r)
            
            if(isobject(r.modbusClient))
                q=r.getRealjoints();

                r.sym_model.teach(q*pi/180);

            else
                r.sym_model.teach();
            end

        end

       function symMove2pos(r,ni,Pose1,Pose2) %Fix

            T1=eye(4);
            T2=eye(4);

            T1(1:3,4)=Pose1(1:3);
            T1(1:3,1:3)=rpy2rotation(Pose1(4)*pi/180,Pose1(5)*pi/180,Pose1(6)*pi/180);

            if(nargin==4)
                
                T2(1:3,4)=Pose2(1:3);
                T2(1:3,1:3)=rpy2rotation(Pose2(4)*pi/180,Pose2(5)*pi/180,Pose2(6)*pi/180);

                T=ctraj(T1,T2,ni);
                q=r.sym_model.ikine(T);

                r.sym_model.plot(q);

            elseif(isobject(r.modbusClient))

                pos=r.getRealpos();

                T2(1:3,4)=pos(1:3);
                T2(1:3,1:3)=rpy2rotation(pos(4)*pi/180,pos(5)*pi/180,pos(6)*pi/180);

                T=ctraj(T2,T1,ni);
                q=r.sym_model.ikine(T);

                r.sym_model.plot(q);

            else
                disp('Not enough input arguments');
            end
        end

%% Gripper functions

        function opengripper(r)

            flush(r.tcpClient);
            
            %Defines flag as HIGH
            write(r.modbusClient,"coils",16,1);
        
            %Exit ListenNode
        
            trama='TMSCT,14,1,ScriptExit(),';
        
            fulltrama=['$' trama '*' checksum(trama)];
        
            writeline(r.tcpClient,fulltrama);
        
            pause(0.5);
        
            flush(r.tcpClient);
        
        end

        function closegripper(r)
    
            flush(r.tcpClient);
            
            %Sets flag as LOW
            write(r.modbusClient,"coils",16,0);
        
            %Exit ListenNode
        
            trama='TMSCT,14,1,ScriptExit(),';
        
            fulltrama=['$' trama '*' checksum(trama)];
        
            writeline(r.tcpClient,fulltrama);
        
            pause(0.5);
        
            flush(r.tcpClient);
        
        end

%% Test arm configuration functions

        function [check]= isjointReach(r,q)

            for i=1:6
                
                if(abs(q(i)) > r.sym_model.qlim(i,2)*180/pi)

                    check=0;
                    return    

                end
            end

            check=1;
        end

        function [check]=isposReach(r,pos)
                
            q=r.I_Kinematics(pos);

            if(length(q)<6)
                check=0;
                return
            elseif (~r.isjointReach(q))
                check=0;
                return
            end
            
            check=1;
        end

    end

end


