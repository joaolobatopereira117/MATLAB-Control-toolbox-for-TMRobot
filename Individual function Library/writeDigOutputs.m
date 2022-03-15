%Write digital outputs
%Inputs: m- Object for the Modbus client;
%        n- number of the digital output to read 0 to 14;
%        value- 0(Low) or 1(High);

function writeDigOutputs(m,n,value)

     if(n>=0)&&(n<=14)
        write(m,"coils",n+1,value);
     else
        disp("Number of output incorrect");
     end

end