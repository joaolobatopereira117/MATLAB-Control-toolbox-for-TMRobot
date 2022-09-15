function [check]= isjointReach(sym_model,q)
    for i=1:6 
        if(abs(q(i)) > sym_model.qlim(i,2)*180/pi)
            check=0;
        return    
        end
    end
        check=1;
end