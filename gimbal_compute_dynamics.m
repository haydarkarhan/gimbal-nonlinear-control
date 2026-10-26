function [M, C, G] = gimbal_compute_dynamics(q, dq, gimbal)
% Compute mass, coriolis and gravity vector

M = zeros(2,2);
C = zeros(2,2);
G = zeros(2,1);

q2 = q(2);
dq1 = dq(1);
dq2 = dq(2);

th = gimbal.params.th;

th1 = th(1);
th2 = th(2);
th3 = th(3);
th4 = th(4);
th5 = th(5);

M11 = th1.*cos(2*q2).^2 + th2;
M12 = th3*sin(q2);
M22 = th4;

C11 = -1/2*th1*sin(2*q2)*dq2;
C12 = -1/2*th1*sin(2*q2)*dq1 + th3*cos(q2)*dq2;
C21 =  1/2*th1*sin(2*q2)*dq1;
C22 = 0;

G1 = 0;
G2 = th5*cos(q2);

M(:,:) = [M11, M12;
          M12, M22];

C(:,:) = [C11, C12;
          C21, C22];

G(:,:) = [G1; G2];