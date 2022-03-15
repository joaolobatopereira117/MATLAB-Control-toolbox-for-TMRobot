%Moves the end effector value mm only in the Z axel 
%Inputs: cliente= TCP/IP Client Object;
%        m= Modbus Object;
%        value in mm;
%        config= "Line" or "PTP";
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed;
%Output: check=Reachable pose or not reachable pose;
%        move= Movement completed or not completed;

function [check,move]=moveZ(client,m,value,config,velperc)

    currentcartesianpose=getcartesianpose(m);

    currentcartesianpose(3)=currentcartesianpose(3)+value;

    %check if reachable pose

    q=zeros(6);

    if(length(q)==6)

        if(config=="Line")

            [check,move]=definecartesianposeLinemove(client,currentcartesianpose,velperc);
        
        elseif(config=="PTP")

            [check,move]=definecartesianposePTPmove(client,currentcartesianpose,velperc);

        else

            check=("Error on Motion config");
        end
    else
        disp("Unreachable pose");
    end
end


