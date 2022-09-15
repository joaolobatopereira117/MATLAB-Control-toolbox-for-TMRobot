function [pos]=getSympos(sym_model) 

    q=getSymjoints(sym_model);
    pos=F_Kinematics(sym_model,q);

 end