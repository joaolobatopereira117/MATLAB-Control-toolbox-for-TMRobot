%Returns robot referential base in reference to his origin base  
%Inputs: m=Object for the Modbus client;
%Output: currBase= base pose [X Y Z Rx Ry Rz] in mm and Degres;
function [currBase]=getBase(m)
    for i=1:6
        currBase(i)=read(m,"inputregs",(8299+i*2),1,1,"single");
    end  
end