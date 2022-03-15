%Moves Joint 3 valueº from actual joint3  
%Inputs: cliente= TCP/IP Client Object;
%        m= Modbus Object;
%        value in degrees;
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed;
%Output: check=Reachable pose or not reachable pose;
%        move= Movement completed or not completed;

function [check,move]=moveJ3(client,m,value,velperc)

    currentjointpose=getjointvalues(m);

    currentjointpose(3)=currentjointpose(3)+value;

    [check,move]=definejointposePTPmove(client,currentjointpose,velperc);
end

