%Returns End effector cartesian pose 
%Inputs: m=Object for the Modbus client;
%Output: pose= cartesian pose [X Y Z Rx Ry Rz] in mm and Degres;
function [pose]=getRealpos(m)
    for i=1:6
        pose(i)=read(m,"inputregs",(7000+i*2),1,1,"single");
    end
end