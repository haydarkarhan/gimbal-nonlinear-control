function gimbal = gimbal_parameters
% Nominal parameters of the system

m1 = 0.051;
m2 = 0.0195;

h1 = [3.014e-5, -0.0227, -0.0214];
h2 = [0.0246, 0, 0.0014];

I1_diag = [4.049e-5, 2.514e-5, 2.084e-5];
I2_diag = [7.4e-7, 3.86e-6, 3.75e-6];

I2_prod = [0, -5.6e-7, 0];

h1y = h1(2);
h2x = h2(1);
h2z = h2(3);
Izz1 = I1_diag(3);
Ixx2 = I2_diag(1);
Iyy2 = I2_diag(2);
Izz2 = I2_diag(2);
Ixz2 = I2_prod(2);
g = -9.80665;

th1 = m2*h2x^2 - Ixx2 + Iyy2;
th2 = m1*h1y^2 + m2*h2z^2 + Izz1 + Ixx2;
th3 = Ixz2 - m2*h2x*h2z;
th4 = m2*h2x^2 + Izz2;
th5 = -m2*h2x*g;

gimbal.params.m1 = m1;
gimbal.params.h1 = h1;
gimbal.params.I1_diag = I1_diag;
gimbal.params.I1_prod = [-8.36e-6, -2.3e-7, 4.6e-7];
gimbal.params.I1_diag_sim = [9.008e-5, 4.846e-5, 4.711e-5];
gimbal.params.I1_prod_sim = [-1.639e-5, 2.6e-7, -4.2e-7];
gimbal.params.b1 = 0;

gimbal.params.m2 = m2;
gimbal.params.h2 = h2;
gimbal.params.I2_diag = I2_diag;
gimbal.params.I2_prod = I2_prod;
gimbal.params.I2_diag_sim = [7.8e-7, 1.571e-5, 1.556e-5];
gimbal.params.I2_prod_sim = [0, -1.1e-7, 0];
gimbal.params.b2 = 0;

gimbal.params.g = g;

gimbal.params.th = [th1; th2; th3; th4; th5];

gimbal.sim.Ts = 0.005;
gimbal.sim.stopTime = 10;
gimbal.sim.activeController = 0;
gimbal.sim.traj.trajSelect = 0;
gimbal.sim.traj.T = 2;
gimbal.sim.robustType = 0;
gimbal.sim.adaptiveType = 0;
gimbal.sim.otherType = 0;
gimbal.sim.movingTarget = 0;
gimbal.sim.hardwareConnection = 0;
gimbal.sim.q1_0 = 0;
gimbal.sim.q2_0 = 0;
gimbal.sim.dq1_0 = 0;
gimbal.sim.dq2_0 = 0;

gimbal.uc.mass = 0;
gimbal.uc.inertia = 0;
gimbal.uc.length = 0;