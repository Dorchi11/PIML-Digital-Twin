clc;
clear;
close all;
%....................................................................
% to run and demonstrate the system with real hardware
%.....................................................................

% to setup the variables
Ts = 0.02;
x0 = [0; 0];
p0 = [0.1; 0.5];

% Digital Twin
dt = DigitalTwin(Ts, x0, p0);

% Arduino connection
port = 'COM3';  % Change to your port
arduino = setupArduino(port);

% Reference
theta_ref = 0.5;

fprintf('\n=== Project 4: Real-Time Digital Twin ===\n');
fprintf('Press Ctrl+C to stop\n\n');

% to run the main loop
try
    k = 1;
    while true
        % Read sensor (Project 2 interface)
        y = readSensor(arduino);
        
        % PID Control (Project 1 interface)
        u = controller.compute(theta_ref, y(1));
        u = max(-12, min(12, u));
        
        % Send to Arduino
        sendControl(arduino, u);
        
        % Update Digital Twin (RLS)
        x_pred = dt.predictState(dt.x, u, dt.p_hat);
        [p_new, ~, ~] = dt.rlsUpdate(x_pred, u, y);
        dt.p_hat = p_new;
        
        % Update state
        dt.x = dt.predictState(dt.x, u, dt.p_hat);
        
        % Store
        dt.x_history = [dt.x_history, dt.x];
        dt.u_history = [dt.u_history, u];
        dt.p_history = [dt.p_history, p_new];
        
        % Display
        if mod(k, 50) == 0
            fprintf('Step %d: θ=%.4f, u=%.4f, damp=%.4f\n', ...
                k, y(1), u, p_new(1));
        end
        
        k = k + 1;
        pause(Ts);
    end
    
catch
    fprintf('\nStopped\n');
    closeArduino(arduino);
end
