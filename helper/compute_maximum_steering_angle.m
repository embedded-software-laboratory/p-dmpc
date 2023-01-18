% CALCULATE THE MAX STEERING ANGLE
Lr = 0.1;
Lf = 0.1;
L = Lr + Lf;
R = 0.3; % MIn. Radius of Curvature

d = sqrt(R^2-Lr^2);
delta = pi/2 - atan(d/L);