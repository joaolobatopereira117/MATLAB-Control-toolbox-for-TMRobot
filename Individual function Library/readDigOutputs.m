%Read Digital outputs
%input: m-Object for the Modbus client
%       n-Number of digital output to read from 0 to 14
%output: state-State of the n pin

function state=readDigOutputs(m,n)

    if(n>=0)&&(n<=14)
     state=read(m,"coils",n+1,1);
    else
     disp("Number of output incorrect");
     state=3;
    end
end