% Reproduce the controller comparison table without Simulink.
%
% Every row integrates the same plant over the same sinusoidal reference
% and reports the RMS tracking error. Rows after the first run against a
% perturbed plant, so the controllers are working from a model that is
% deliberately wrong.

cases = { "emk",            false, 'Exact model knowledge';
          "emk",            true,  'Exact model knowledge (uncertainty)';
          "high gain",      true,  'Robust, high gain';
          "high frequency", true,  'Robust, high frequency';
          "sliding mode",   true,  'Robust, sliding mode';
          "adaptive",       true,  'Adaptive, standard';
          "dmc",            true,  'Adaptive, desired model comp.' };

runs = cell(size(cases,1), 1);
rows = zeros(size(cases,1), 4);

for i = 1:size(cases,1)
    runs{i} = simulate_gimbal(cases{i,1}, 'Uncertainty', cases{i,2});
    rows(i,:) = runs{i}.rms;
end

T = array2table(rows, ...
    'VariableNames', {'e1','e2','de1','de2'}, ...
    'RowNames', cases(:,3));
T.Properties.Description = 'RMS tracking error over a 10 s run';
disp(T)
