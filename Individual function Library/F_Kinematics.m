function [Pose] = F_Kinematics(sym_model,q)
            
            posMatrix=sym_model.fkine(q*pi/180);

            Pose(1:3)=posMatrix.t';
            
            rotMatrix=[posMatrix.n posMatrix.o posMatrix.a];

            [Rx,Ry,Rz]=rotation2rpy(rotMatrix);
            
            Pose(4)=Rx*180/pi;
            Pose(5)=Ry*180/pi;
            Pose(6)=Rz*180/pi;
                
        end