function [check]=isposReach(sym_model,pos)
        
    q=I_Kinematics(sym_model,pos);

    if(length(q)<6)
        check=0;
        return
    elseif (~isjointReach(sym_model,q))
        check=0;
        return
    end
    
    check=1;
end
