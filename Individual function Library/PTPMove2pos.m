%Moves the end effector to desired cartesian pose with PTP movement
%trajectorie;

%Inputs: cliente= TCP/IP Client Object;
%        pose=(x y z rx ry rz)in mm and degres;
%        velperc= movement velocity in %, robo will take 100 ms to get to
%        full speed;

%Output: check=Valid comand or invalid comand;
%        move= Movement completed or not completed;

function [check,move]=PTPMove2pos(client,pose,velperc)

    flush(client);
    

    data=['1,PTP("CPP",' num2str(pose(1)) ',' num2str(pose(2)) ',' num2str(pose(3)) ',' num2str(pose(4)) ',' num2str(pose(5)) ',' num2str(pose(6)) ',' num2str(velperc) ',100,0,false)' char(13) newline 'QueueTag(1)'];

    headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];

    fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];

    writeline(client,fullcomand);

    message=char(readline(client));

    if(message(12:13) == "OK")
        check='Valid_comand';
    else
        check='Invalid_comand';
        move='Movement not completed';
        return;
    end

    while(client.NumBytesAvailable==0)
    end

    message=char(readline(client));

    if(message(17:20)=="true")
        move='Movement executed';

    else
        move='Movement not completed';
    end
end