%%This function requires extra nodes on the TMflow
%project for this specific
%gripper;

%Opens gripper fingers  
%Inputs: cliente= TCP/IP Client Object;
%        m-Object for the Modbus client

function opengripper(m,client)

    flush(client);
    
    %Defines flag as HIGH
    write(m,"coils",16,1);

    %Exit ListenNode

    trama='TMSCT,14,1,ScriptExit(),';

    fulltrama=['$' trama '*' checksum(trama)];

    writeline(client,fulltrama);

    pause(0.5)

    flush(client);

end