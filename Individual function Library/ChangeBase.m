%Changes robot base, after changing robots base all movements will be done
%acording to this newbase;

%Inputs: client= TCP/IP Client Object;
%        newbase=(X Y Z Rx Ry Rz)in mm and degres;
%Output: check=Checking if base was changed sucssesfully;


function check=ChangeBase(client,newbase)

       flush(client);

        data=['1,ChangeBase(' num2str(newbase(1)) ',' num2str(newbase(2)) ',' num2str(newbase(3)) ',' num2str(newbase(4)) ',' num2str(newbase(5)) ',' num2str(newbase(6)) ')'];

        headerplusdata=['TMSCT,' num2str(length(data)) ',' data ','];
    
        fullcomand=['$' headerplusdata '*' checksum(headerplusdata) ];
   
        writeline(client,fullcomand);

        message=char(readline(client));

        if(message(12:13) == "OK")
            check='Base changed';
        else
            check='Base change error';
        end
     
end