%This function is only for watching the robot's virtual model following the
%real robot wich would be controlled thru its own software tmflow;

mdl_tm5_700_Robo; %Creates Robot model

m=modbus("tcpip","192.168.0.111",502);

while(1)
    for i=1:6
        q(i)=(read(m,"inputregs",(7012+i*2),1,1,"single")*pi/180);
    end
    tm5_700.plot(q);
end