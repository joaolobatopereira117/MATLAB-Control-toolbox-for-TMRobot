%Read Digital inputs
%input: m-Object for the Modbus client
%       n-Number of digital input to read from 0 to 15
%output:state- state of the n pin 

function state=getDI(m,n)

    if(n>=0)&&(n<=15)
     state=read(m,"inputs",n+1,1);
    else
     disp("Number of input incorrect");
     state=3;
    end
end