%Moves Joint 6 valueº from actual joint6  
%Inputs: cliente= TCP/IP Client Object;
%        m= Modbus Object;
%        value in degrees;
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed;
%Output: check=Reachable pose or not reachable pose;
%        move= Movement completed or not completed;

function [check,move]=moveJ6(client,m,value,velperc)

    
    currentjointpose=getjointvalues(m);

    currentjointpose(6)=currentjointpose(6)+value;

    [check,move]=definejointposePTPmove(client,currentjointpose,velperc);

end

