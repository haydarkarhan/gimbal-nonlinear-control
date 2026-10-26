% Open the Simulink model and its front end.
%
% The app sets the controller, the trajectory and the parameter uncertainty.
% The model reads those choices through variant subsystems, so the app has to
% be running before the model will simulate.
%
% Try the "Moving Target" slider: it switches the reference to a task-space
% target and converts it to joint space on the fly.
%
% gimbal_dynamics.mlx contains the symbolic derivation of the dynamics.
%
% WARNING: simulating this model opens Mechanics Explorer, which renders the
% 3-D animation on the GPU. On a laptop with switchable graphics that has been
% known to hang the display driver. For the numbers alone, use
% simulate_gimbal.m instead, which needs neither Simulink nor Simscape:
%
%     compare_controllers
%     result = simulate_gimbal("high gain", 'Uncertainty', true);

run("gimbal_app.mlapp")
open_system("gimbal_simulink");
