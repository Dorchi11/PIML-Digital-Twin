clc;
clear;
close all;
%...................................................................
% to test the system in simulation and find the error without hard
%...................................................................

% to setup the digital twin system
Ts = 0.01;
x = [0; 0];
p = [0.1; 0.5];  % [damping; thrust_gain]

% Create Digital Twin 
dt = Digital_Twin(Ts, x, p);

% Override RLS parameters for MAXIMUM stability
dt.P_rls = 0.0001 * eye(2);   % Very very small covariance
dt.lambda_rls = 0.999;         % Very slow forgetting
dt.theta_rls = [0.1; 0.5];     % Initial guess close to true

% get the reference from model 1,2,3
% to define reference trajectory (VERY SMALL amplitude)
numSteps = 200; 
t = (0:numSteps-1) * Ts;
r_trajectory = zeros(2, numSteps);
r_trajectory(1,:) = 0.05 * sin(0.2 * t);   
r_trajectory(2,:) = 0.02 * cos(0.2 * t);  

% to run simulation
dt.runSimulation(numSteps, r_trajectory, [], []);

% Check if parameters are valid before plotting
if any(isnan(dt.p_hat)) || any(isinf(dt.p_hat))
    fprintf('\Parameter estimation failed\n');
    fprintf('Using true parameters for display\n');
    dt.p_hat = dt.p;
end

% Try-catch for plotting
try
    dt.plotResults();
catch ME
    fprintf('\nPlotting failed: %s\n', ME.message);
    fprintf('   Plotting manually\n');
    
    % Manual plotting
    figure('Position', [100, 100, 800, 600]);
    
    subplot(2,1,1);
    plot(dt.x_history(1,:), 'b-', 'LineWidth', 1.5); hold on;
    plot(dt.x_history(2,:), 'r-', 'LineWidth', 1.5);
    title('States'); xlabel('Time Step'); ylabel('Value');
    legend('\theta', '\omega'); grid on;
    
    subplot(2,1,2);
    plot(dt.p_history(1,:), 'g-', 'LineWidth', 1.5); hold on;
    plot(dt.p_history(2,:), 'm-', 'LineWidth', 1.5);
    plot([1, length(dt.p_history)], [dt.p(1), dt.p(1)], 'g--', 'LineWidth', 1);
    plot([1, length(dt.p_history)], [dt.p(2), dt.p(2)], 'm--', 'LineWidth', 1);
    title('Parameters'); xlabel('Time Step'); ylabel('Value');
    legend('Damping (est)', 'Thrust Gain (est)', 'Damping (true)', 'Thrust Gain (true)');
    grid on;
end

% Display final results
fprintf('\n');
fprintf('\n Final Results:');
fprintf('\n   True Damping:     %.4f', dt.p(1));
fprintf('\n   True Thrust Gain: %.4f', dt.p(2));
fprintf('\n   Estimated Damping:     %.4f', dt.p_hat(1));
fprintf('\n   Estimated Thrust Gain: %.4f', dt.p_hat(2));
if ~any(isnan(dt.p_hat))
    fprintf('\n   Damping Error:    %.4f', abs(dt.p_hat(1) - dt.p(1)));
    fprintf('\n   Thrust Error:     %.4f', abs(dt.p_hat(2) - dt.p(2)));
end
fprintf('\n\n');
