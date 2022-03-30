function [T] = transf_matrix (alpha, a, theta_i, d_i)
%Performes Homogeneus trasnformation matrix

T = [cos(theta_i)            -sin(theta_i)             0           a;
     sin(theta_i)*cos(alpha) cos(theta_i)*cos(alpha) -sin(alpha) -sin(alpha)*d_i;
     sin(theta_i)*sin(alpha) cos(theta_i)*sin(alpha) cos(alpha)  cos(alpha)*d_i;
     0                       0                       0           1             ];
end