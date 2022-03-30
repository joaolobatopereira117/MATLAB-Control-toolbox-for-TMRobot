function [q] = I_Kinematics(sym_model,Pose)

            posMatrix=transl(Pose(1),Pose(2),Pose(3))*trotx(Pose(4)*pi/180)*troty(Pose(5)*pi/180)*trotz(Pose(6)*pi/180);

            j=sym_model.ikcon(posMatrix);

            q=j*180/pi;
                
end
