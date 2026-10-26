function result = simulate_gimbal(controller, varargin)
% Closed-loop simulation of the 2-DoF gimbal, without Simulink
%
%   result = simulate_gimbal(controller) integrates the joint-space
%   dynamics M(q)qdd + C(q,qd)qd + G(q) = tau under the named controller
%   and returns the tracking history.
%
%   Supported controllers:
%     "emk"            exact model knowledge
%     "high gain"      robust, VR = rho/eps * r
%     "high frequency" robust, VR = rho^2/(rho*norm(r) + eps) * r
%     "sliding mode"   robust, VR = rho*sign(r)
%     "adaptive"       standard adaptive, thetahat_dot = Gamma*Y'*r
%     "dmc"            adaptive with desired model compensation
%
%   Name-value options:
%     Uncertainty  true/false, perturb the plant parameters (default false)
%     StopTime     horizon in seconds (default 10)
%     Ts           output sample period (default 0.005)
%     Period       trajectory period T in seconds (default 2)
%     Amplitude    trajectory amplitude in rad (default 0.4)
%     Solver       "auto", "ode15s" or "rk4" (default "auto")
%     BoundaryLayer  smooth sign(r) as r./sqrt(r.^2 + phi) (default 0, off)
%     MaxStep        internal RK4 step, substepped below Ts (default 1e-5)
%
%   Sliding mode switches on sign(r), which has no Jacobian, so a variable
%   step solver stalls on it at once. "auto" therefore integrates that one
%   with fixed-step RK4 substepped to MaxStep and leaves every continuous
%   controller on ode15s. A non-zero BoundaryLayer replaces the switch with
%   a smooth saturation, which removes the chattering and lets ode15s run.
%
%   The plant integrates the perturbed parameters while every controller
%   builds its model terms from the nominal ones, which is the mismatch the
%   robust and adaptive designs exist to handle.

controller = string(controller);

opts = struct('Uncertainty', false, 'StopTime', 10, 'Ts', 0.005, ...
              'Period', 2, 'Amplitude', 0.4, 'Solver', "auto", ...
              'BoundaryLayer', 0, 'MaxStep', 1e-5);
for k = 1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k+1};
end

gimbal = gimbal_parameters;

% Controller model: always the nominal parameters.
ctrlModel = gimbal;
ctrlModel.params.th = gimbal_theta(gimbal);

% Plant model: perturbed when uncertainty is switched on. The offsets match
% the ones used in the report.
plantModel = gimbal;
if opts.Uncertainty
    uc = struct('mass', 0.014, 'inertia', 5e-6, 'length', 0.0075);
else
    uc = struct('mass', 0, 'inertia', 0, 'length', 0);
end
plantModel.params.th = gimbal_theta(gimbal, uc);

alpha = diag([1 1]);
isAdaptive = ismember(controller, ["adaptive", "dmc"]);

if isAdaptive
    Kr = diag([10 10]);
    Gamma = diag([5 5 5 5 5]);
    % The adaptation integrator starts from the nominal parameters.
    z0 = [0; 0; 0; 0; ctrlModel.params.th];
elseif controller == "emk"
    Kr = diag([2 2]);
    Gamma = [];
    z0 = [0; 0; 0; 0];
else
    Kr = diag([10 10]);
    Gamma = [];
    z0 = [0; 0; 0; 0];
end

tGrid = (0:opts.Ts:opts.StopTime)';

switch lower(string(opts.Solver))
    case "auto",   useRK4 = (controller == "sliding mode") && opts.BoundaryLayer == 0;
    case "rk4",    useRK4 = true;
    case "ode15s", useRK4 = false;
    otherwise, error('simulate_gimbal:solver', 'Unknown solver "%s".', opts.Solver);
end

if useRK4
    nSub = max(1, ceil(opts.Ts / opts.MaxStep));
    [t, z] = rk4(@dynamics, tGrid, z0, nSub);
else
    odeOpts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    [t, z] = ode15s(@dynamics, tGrid, z0, odeOpts);
end

q  = z(:, 1:2);
dq = z(:, 3:4);

% Replay the controller along the solution to recover the signals that are
% not states: the reference, the errors and the torque.
n = numel(t);
[qd, dqd, tau] = deal(zeros(n, 2));
for i = 1:n
    [qd(i,:), dqd(i,:)] = deal_traj(t(i));
    if isAdaptive
        tau(i,:) = control(t(i), q(i,:)', dq(i,:)', z(i,5:9)')';
    else
        tau(i,:) = control(t(i), q(i,:)', dq(i,:)', [])';
    end
end

result = struct();
result.controller = controller;
result.uncertainty = opts.Uncertainty;
result.t = t;
result.q = q;
result.dq = dq;
result.q_d = qd;
result.dq_d = dqd;
result.e = qd - q;
result.de = dqd - dq;
result.tau = tau;
if isAdaptive
    result.theta_hat = z(:, 5:9);
    result.theta_true = plantModel.params.th';
end
result.rms = sqrt(mean([result.e, result.de].^2, 1));

    function dz = dynamics(t, z)
        qq  = z(1:2);
        dqq = z(3:4);
        if isAdaptive
            thHat = z(5:9);
            [tauNow, dth] = control(t, qq, dqq, thHat);
        else
            tauNow = control(t, qq, dqq, []);
            dth = [];
        end
        [M, C, G] = gimbal_compute_dynamics(qq, dqq, plantModel);
        ddq = M \ (tauNow - C*dqq - G);
        dz = [dqq; ddq; dth];
    end

    function [tau, dth] = control(t, q, dq, thHat)
        [qdes, dqdes, ddqdes] = reference(t);

        e  = qdes - q;
        de = dqdes - dq;
        r  = de + alpha*e;

        dth = [];

        switch controller
            case "emk"
                [M, C, G] = gimbal_compute_dynamics(q, dq, ctrlModel);
                tau = M*(ddqdes + alpha*de) + C*(dqdes + alpha*e) + G + Kr*r;

            case "high gain"
                N = nominalN(q, dq, e, de, dqdes, ddqdes);
                rho = 1; eps_ = 0.01;
                tau = N + Kr*r + rho/eps_*r;

            case "high frequency"
                N = nominalN(q, dq, e, de, dqdes, ddqdes);
                rho = 1; eps_ = 0.01;
                tau = N + Kr*r + rho^2/(rho*norm(r) + eps_)*r;

            case "sliding mode"
                N = nominalN(q, dq, e, de, dqdes, ddqdes);
                rho = diag([0.1 0.1]);
                if opts.BoundaryLayer > 0
                    sw = r ./ sqrt(r.^2 + opts.BoundaryLayer);
                else
                    sw = sign(r);
                end
                tau = N + Kr*r + rho*sw;

            case "adaptive"
                Y = regressor(q(2), dq(1), dq(2), ddqdes + alpha*de, dqdes + alpha*e);
                dth = Gamma*Y.'*r;
                tau = Kr*r + Y*thHat;

            case "dmc"
                % The regressor is evaluated along the desired trajectory,
                % so it can be computed offline.
                Yd = regressor(qdes(2), dqdes(1), dqdes(2), ddqdes, dqdes);
                dth = Gamma*Yd.'*r;
                rho = 0.1;
                tau = Kr*r + Yd*thHat + rho*norm(r);

            otherwise
                error('simulate_gimbal:unknownController', ...
                      'Unknown controller "%s".', controller);
        end
    end

    function N = nominalN(q, dq, e, de, dqdes, ddqdes)
        [M, C, G] = gimbal_compute_dynamics(q, dq, ctrlModel);
        N = M*(ddqdes + alpha*de) + C*(dqdes + alpha*e) + G;
    end

    function [qdes, dqdes, ddqdes] = reference(t)
        A = opts.Amplitude;
        w = 2*pi/opts.Period;
        qdes   = [A*cos(w*t) - A;  A*cos(w*t) - A];
        dqdes  = [-A*w*sin(w*t);  -A*w*sin(w*t)];
        ddqdes = [-A*w^2*cos(w*t); -A*w^2*cos(w*t)];
    end

    function [qd_, dqd_] = deal_traj(t)
        [a, b] = reference(t);
        qd_ = a'; dqd_ = b';
    end
end

function Y = regressor(q2, dq1, dq2, u, v)
% Linear-in-the-parameters regressor: Y*theta reproduces M*u + C*v + G.
u1 = u(1); u2 = u(2);
v1 = v(1); v2 = v(2);

Y = [u1*cos(q2)^2 - 1/2*sin(2*q2)*(v1*dq2 + v2*dq1), u1, u2*sin(q2) + v2*cos(q2)*dq2, 0,  0;
     1/2*v1*sin(2*q2)*dq1,                            0,  u1*sin(q2),                 u2, cos(q2)];
end

function [t, z] = rk4(f, t, z0, nSub)
% Fixed-step classical Runge-Kutta, for right-hand sides that switch.
% Takes nSub internal steps per output interval, so the switching stays
% resolved without returning a million samples.
z = zeros(numel(t), numel(z0));
z(1,:) = z0';
y = z0;
for i = 1:numel(t)-1
    h = (t(i+1) - t(i)) / nSub;
    tt = t(i);
    for s = 1:nSub
        k1 = f(tt,       y);
        k2 = f(tt + h/2, y + h/2*k1);
        k3 = f(tt + h/2, y + h/2*k2);
        k4 = f(tt + h,   y + h*k3);
        y = y + h/6*(k1 + 2*k2 + 2*k3 + k4);
        tt = tt + h;
    end
    z(i+1,:) = y';
end
end
