function symJoinTraj(sym_model,jpos1,jpos2,t)

    tmp=[0:0.1:t]';
    qtraj=jtraj(jpos1*pi/180,jpos2*pi/180,tmp);
    sym_model.plot(qtraj);

end