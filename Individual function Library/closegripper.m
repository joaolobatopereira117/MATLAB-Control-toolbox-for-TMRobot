%This function requires extra nodes on the TMflow project for this specific
%gripper;

%Closes gripper fingers  
%Inputs: cliente= TCP/IP Client Object;
%        m-Object for the Modbus client    

function closegripper(m,client)
    
    flush(client);
    
    %Sets flag as LOW
    write(m,"coils",16,0);

    %Exit ListenNode

    trama='TMSCT,14,1,ScriptExit(),';

    fulltrama=['$' trama '*' checksum(trama)];

    writeline(client,fulltrama);

    pause(0.5);

    flush(client);

end