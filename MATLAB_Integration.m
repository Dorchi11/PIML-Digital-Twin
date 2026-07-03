%..................................................
% to communicate the matlab with arduino
%...................................................

% --- Setup Serial Communication ---
try
    arduino_port = 'COM3';  % Change to your port
    s = serialport(arduino_port, 115200);
    configureTerminator(s, "LF");
    fprintf('Connected to Arduino on %s\n', arduino_port);
catch
    error('Could not connect to Arduino. Check port!');
end

% --- Digital Twin Object ---
Ts = 0.01;  % 10ms sampling
x0 = [0; 0];
p0 = [0.5; 1.0];  % [damping; thrust_gain]
dt = DigitalTwin(Ts, x0, p0);

% --- Reference Trajectory ---
t = 0:Ts:10;
r_trajectory = [0.5*sin(0.5*t); 0.5*cos(0.5*t)];

% --- Real-time Loop ---
numSteps = 1000;
for k = 1:numSteps
    % 1. Read from Arduino
    if s.NumBytesAvailable > 0
        data_line = readline(s);
        data = str2double(strsplit(data_line, ','));
        
        if length(data) == 3
            % Parse measurements
            theta_meas = data(1);
            omega_meas = data(2);
            y_meas = [theta_meas; omega_meas];
            
            % 2. Update Digital Twin
            x_pred = dt.predictState(dt.x, dt.u_history(end), dt.p_hat);
            [p_new, ~, ~] = dt.rlsUpdate(x_pred, dt.u_history(end), y_meas);
            dt.p_hat = p_new;
            
            % 3. Compute Control (PID or MPC)
            r = r_trajectory(:, k);
            u = -10*y_meas(1) - 5*y_meas(2);  % Simple PD control
            
            % 4. Send control to Arduino
            writeline(s, num2str(u));
            
            % 5. Update Digital Twin state
            dt.x = dt.predictState(dt.x, u, dt.p_hat);
            
            % 6. Store data
            dt.x_history = [dt.x_history, dt.x];
            dt.u_history = [dt.u_history, u];
            dt.p_history = [dt.p_history, p_new];
        end
    end
    
    % Real-time timing
    pause(Ts);
end

% --- Cleanup ---
clear s;
fprintf('Real-time simulation completed!\n');

% --- Plot Results ---
dt.plotResults();
