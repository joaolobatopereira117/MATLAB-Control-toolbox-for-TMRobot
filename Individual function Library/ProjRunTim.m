%Project running time since last time Play pressed 
%Inputs: m=Object for the Modbus client;
%Output: RunTim=[Days Hours Minutes Seconds];

function RunTim=ProjRunTim(m)
       q=1;
       for i=1:5
        if(i~=2)
            RunTim(q)=read(m,"inputregs",8200+i,1);
            q=q+1;
        end
       end

       display=['Project running for ' num2str(RunTim(1)) ' days ' num2str(RunTim(2)) ' hours ' num2str(RunTim(3)) ' minutes and ' num2str(RunTim(4))  ' seconds'];

       disp(display);
        
end