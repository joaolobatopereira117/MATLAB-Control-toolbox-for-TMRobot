function symControl(sym_model,q)            
       if(nargin==2)
            sym_model.teach(q*pi/180);
        else
            sym_model.teach();
        end
end