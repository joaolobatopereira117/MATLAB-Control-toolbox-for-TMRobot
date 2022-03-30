function [currBase]=getBase(m)

            for i=1:6
                currBase(i)=read(m,"inputregs",(8299+i*2),1,1,"single");
            end  

end