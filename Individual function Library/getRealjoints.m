%Returns instant joint values 
%Inputs: m=Object for the Modbus client;
%Output: q= Joint values in degres;
function [q]=getRealjoints(m)
    for i=1:6
        q(i)=read(m,"inputregs",(7012+i*2),1,1,"single");
    end
end