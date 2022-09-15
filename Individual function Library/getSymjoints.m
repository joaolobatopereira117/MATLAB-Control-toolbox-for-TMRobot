function [q]=getSymjoints(sym_model)      

    q=sym_model.getpos()*180/pi;

end