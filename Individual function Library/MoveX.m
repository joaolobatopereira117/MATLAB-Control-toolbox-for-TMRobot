%Moves the end effector value mm only in the X axel 
%Inputs: cliente= TCP/IP Client Object;
%        m= Modbus Object;
%        value in mm;
%        config= "Line" or "PTP";
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed;
%Output: check=Reachable pose or not reachable pose;
%        move= Movement completed or not completed;

function [check,move]=MoveX(client,m,value,config ,velperc)

    currentcartesianpose=getRealpos(m);

    currentcartesianpose(1)=currentcartesianpose(1)+value;

        if(config=="Line")

            [check,move]=LineMove2pos(client,currentcartesianpose,velperc);
        
        elseif(config=="PTP")

            [check,move]=PTPMove2pos(client,currentcartesianpose,velperc);

        else

            check=("Error on Motion config");
        end
end


