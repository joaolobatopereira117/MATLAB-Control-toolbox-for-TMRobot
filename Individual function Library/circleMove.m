%Moves the end effector in a circular movement from actual point thru Passpoint and Endpoint with arcAng

%Inputs: cliente= TCP/IP Client Object;
%        m= Modbus client Object;
%        Endpose=(x y z rx ry rz)in mm and degres;
%        Passpose=(x y z rx ry rz)in mm and degres;
%        arcAng in degres;
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed

%Output: check=Possible movement or impossible movement;
%        move= Movement completed or not completed;

function [check,move]=circleMove(client,m,Endpose,Passpose,arcAng,velperc)

    flush(client);

    Initpose=getRealpos(m);

    c=0;
    x=0;
    y=0;
    z=0;
    q=1;

    for i=1:6
        if(Initpose(i)==Endpose(i) || Initpose(i)==Passpose(i))
            c=c+1;
        end
    end
    
    if(Initpose(1)==Endpose(1) && Initpose(1)==Passpose(1))
        x=1;
    end

    if(Initpose(2)==Endpose(2) && Initpose(2)==Passpose(2))
        y=1;
    end

    if(Initpose(3)==Endpose(3) && Initpose(3)==Passpose(3))
        z=1;
    end

    if(c==6 || (x+y+z)>=2)
        q=0;
    end

    if(q==1)

        data=['1,Circle("CPP",' num2str(Passpose(1)) ',' num2str(Passpose(2)) ',' num2str(Passpose(3)) ',' num2str(Passpose(4)) ',' num2str(Passpose(5)) ',' num2str(Passpose(6)) ',' num2str(Endpose(1)) ',' num2str(Endpose(2)) ',' num2str(Endpose(3)) ',' num2str(Endpose(4)) ',' num2str(Endpose(5)) ',' num2str(Endpose(6)) ',' num2str(velperc) ',100,0,' num2str(arcAng) ' ,false)' char(13) newline 'QueueTag(1)'];
            
        headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
    
        fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
    
        writeline(client,fullcomand);
    
        message=char(readline(client));
    
        if(message(12:13) == "OK")
            check='Valid_comand';
        else
            check='Invalid_comand';
        end
    
        while(r.tcpClient.NumBytesAvailable==0)
        end
    
        message=char(readline(client));
    
        if(message(17:20)=="true")
            move='Movement executed';
    
        else
            move='Movement not completed';
        end
    
    else
        disp("Unreachable pose");
        check=0;
        move=0;
    end
end