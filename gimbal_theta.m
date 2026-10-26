function th = gimbal_theta(gimbal, uc)
% Lumped dynamic parameters, with optional parameter uncertainty
%
%   th = gimbal_theta(gimbal) reproduces the nominal gimbal.params.th.
%
%   th = gimbal_theta(gimbal, uc) offsets every mass by uc.mass, every
%   moment of inertia by uc.inertia and every centre-of-mass coordinate by
%   uc.length before lumping. That is how the Simscape plant injects
%   parameter uncertainty, so this is the plant the controllers are made to
%   fight when uncertainty is switched on.

if nargin < 2
    uc = struct('mass', 0, 'inertia', 0, 'length', 0);
end

p = gimbal.params;

m1 = p.m1 + uc.mass;
m2 = p.m2 + uc.mass;

h1 = p.h1 + uc.length;
h2 = p.h2 + uc.length;

I1_diag = p.I1_diag + uc.inertia;
I2_diag = p.I2_diag + uc.inertia;

h1y = h1(2);
h2x = h2(1);
h2z = h2(3);

Izz1 = I1_diag(3);
Ixx2 = I2_diag(1);
Iyy2 = I2_diag(2);
Izz2 = I2_diag(2);
Ixz2 = p.I2_prod(2);

g = p.g;

th = [ m2*h2x^2 - Ixx2 + Iyy2;
       m1*h1y^2 + m2*h2z^2 + Izz1 + Ixx2;
       Ixz2 - m2*h2x*h2z;
       m2*h2x^2 + Izz2;
      -m2*h2x*g ];
