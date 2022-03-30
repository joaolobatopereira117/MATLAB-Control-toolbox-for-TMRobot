%Project speed in percentage 
%Inputs: m=Object for the Modbus client;
%Output: projSpeed= Projest speed in percent according to the safety defined Max speed;

function projSpeed=getprojSpeed(m)

    projSpeed=read(m,"inputregs",7102,1);
    
end