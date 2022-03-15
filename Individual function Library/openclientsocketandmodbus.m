%Opens client socket comunication with ListenNode and creates modbus
%client;
%Inputs: ip=IP adress of socket server and modbus server;
%        port=Number of the comunication Port normally 5890;
%Output: cliente= TCP/IP Client Object;
%        m= Modbus client at port 502        


function [cliente,m]=openclientsocketandmodbus(ip,port)
    
    m=modbus("tcpip",ip,502);

    client=tcpclient(ip,port);

    client.configureTerminator("CR/LF");

    flush(client);

    writeline(client,"$TMSTA,2,00,*41");

    response=char(readline(client));

    if(response(14:17)=="true")

        cliente=client;
        disp("Connected to Listen Node in External script control");

    elseif(response(13:17)=="false")
        
        disp("Not in external script control");

    end

end