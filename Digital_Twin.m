classdef Digital_Twin < handle
% To design the algorithms for RLS and MPC
%...........................................................
    properties
        % Time parameters
        Ts = 0.01;          % Sampling time (s)
        t_current = 0;      % Current simulation time (s)
        
        % State parameters [theta; omega]
        x = [0; 0];         % Current state
        x_history = [0; 0]; % Store states history
        u_history = 0;      % Store control inputs history
        p_history = [0.5; 1.3]; % Store parameter history
        
        % True parameters (for validation)
        p = [0.5; 1.3];     % [damping; thrust_gain]
        p_hat = [0.5; 1.3]; % Estimated parameters (RLS)
        
        % RLS Algorithm Parameters
        theta_rls = [0.5; 1.3];   % RLS parameter vector
        P_rls = 10 * eye(2);    % RLS covariance matrix
        lambda_rls = 0.99;        % Forgetting factor
        
        % MPC Parameters
        Np = 10;            % Prediction horizon
        Nc = 3;             % Control horizon
        Q = diag([10, 1]);  % State weighting matrix
        R = 0.1;            % Input weighting matrix
        u_min = -300;       % Min control input
        u_max = 300;        % Max control input
        
        % Physical constants
        J = 0.01;           % Moment of inertia
    end

    methods
        %.............................................................
        % Constructor: Initialize the Digital Twin
        
        function obj = Digital_Twin(varargin)
            % to convert mathematical models into real-time actions
            % to set all variables
            if nargin >= 1
                obj.Ts = varargin{1};
            end
            if nargin >= 2
                obj.x = varargin{2};
                obj.x_history = varargin{2};
            end
            if nargin >= 3
                obj.p = varargin{3};
                obj.p_hat = varargin{3};
                obj.theta_rls = varargin{3};
                obj.p_history = varargin{3};
            end
        end
        
        %.............................................................
        % 1. Build Simulator: State Prediction 
        % x_{k+1} = x_k + T_s * f(x_k, u_k, p)
      
        function x_next = predictState(obj, x, u, p)
            % State Prediction physical model
            % to predict and update the time using mathematical model
            % to calculate the difference between actual measurement
            
            % Unpack states
            theta = x(1);
            omega = x(2);
            
            % Unpack parameters
            damping = p(1);
            thrust_gain = p(2);
            
            % 2. Integrate Subsystems: Hybrid Thrust 
            thrust = obj.hybridThrustModel(u, thrust_gain);
            
            % Dynamics (Physics-informed model)
            theta_dot = omega;
            omega_dot = (1/obj.J) * (thrust - damping * omega);
            
            % Euler integration
            theta_next = theta + obj.Ts * theta_dot;
            omega_next = omega + obj.Ts * omega_dot;
            
            x_next = [theta_next; omega_next];
        end
        
        %.............................................................
        % 2. Interface: Hybrid Thrust Model (3)
       
        function thrust = hybridThrustModel(~, u, thrust_gain)
            % Interface to Hybrid Thrust 
            % Replace with actual function when ready
            
            % Simple physics model (placeholder)
            thrust = thrust_gain * u^2;
            
            % Uncomment when 3 is ready:
            % params.thrust_gain = thrust_gain;
            % thrust = hybridThrustModel(u, params);
        end
        
        %.............................................................
        % 3. Enable Adaptation: RLS Parameter Estimation
        %    Recursive Least Squares for online parameter adaptation
       
        function [obj, theta_new, P_new] = rlsUpdate(obj, x, u, y)
            % RLS Parameter Adaptation function
            % to compute the adaptive gain vector
            % to apply the forgetting factor
            
            % Get angular velocity from state
            omega = x(2);
            
            % Sensitivity matrix (Jacobian w.r.t parameters)
            psi = zeros(2, 2);
            psi(2, 1) = -omega / obj.J;    % Sensitivity to damping
            psi(2, 2) = u^2 / obj.J;       % Sensitivity to thrust_gain
            
            % Prediction error (innovation) - y is measured output
            error = y - x;

           % Regularization for numerical stability
            reg = 0.1;
            
            % Kalman gain for RLS
            denom = psi * obj.P_rls * psi' + reg + 1;
            K = obj.P_rls * psi' / denom;

            % Update parameters
            theta_new = obj.theta_rls + K * error;
            
            % Update covariance (with forgetting factor)
            P_new = (1/obj.lambda_rls) * (obj.P_rls - K * psi * obj.P_rls);
            
            % Ensure symmetry and positive definiteness
            P_new = (P_new + P_new') / 2;

            % Add regularization
            P_new = P_new + reg * eye(2);
    
            % Clamp P_rls to prevent explosion
            P_new = min(P_new, 0.1 * eye(2));
            P_new = max(P_new, 0.0001 * eye(2))
    
            
            % Store for next iteration
            obj.theta_rls = theta_new;
            obj.P_rls = P_new;
            obj.p_hat = theta_new;
        end
              
        %.............................................................
        % 4. Real-Time Control: Model Predictive Control (MPC)
        %    to calculate the optimal future control sequence
        %    to apply the constraints
       
       function u_opt = mpcCompute(obj, x0, r)
        % Model Predictive Control function
    
        % Initial guess for control sequence
        u0 = zeros(obj.Nc, 1);
    
        % Bounds
        lb = obj.u_min * ones(obj.Nc, 1);
        ub = obj.u_max * ones(obj.Nc, 1);
    
        % Solve optimization using fmincon
        options = optimoptions('fmincon', ...
        'Display', 'off', ...
        'Algorithm', 'sqp', ...
        'MaxIterations', 50, ...
        'TolFun', 1e-6, ...
        'TolX', 1e-6);
    
       [u_opt, ~] = fmincon(@(u) obj.mpcCost(x0, u, r), ...
        u0, [], [], [], [], lb, ub, ...
        @(u) obj.mpcConstraints(x0, u), options);
    
        % Return only the first control input
        u_opt = u_opt(1);
    end

        
        %.............................................................
        % MPC Cost Function
        % to assign numerical weights for control goal
    
        function J = mpcCost(obj, x0, u_seq, r)
            J = 0;
            x = x0;
            
            for k = 1:obj.Np
                if k <= obj.Nc
                    u = u_seq(k);
                else
                    u = u_seq(end);
                end
                
                % to predict next state
                x = obj.predictState(x, u, obj.p_hat);

                % Check for numerical issues
                if any(isnan(x)) || any(isinf(x))
                    J = 1e6;  % Large penalty for invalid states
                    return;
                end

                % to compute error if reference exists
                if k <= size(r, 2)
                    e = x - r(:, k);
                    J = J + e' * obj.Q * e + obj.R * u^2;
                end
            end
        end
        
        %.............................................................
        % MPC Constraints
        % to define the constraints for mpc
       
        function [c, ceq] = mpcConstraints(~, ~, ~)
            c = [];
            ceq = [];
        end
        
        %.............................................................
        % 4. Real-Time Control: Main Simulation Loop
        % to ensure prediction, optimizations, and parameter updates
        % to allow RLS, MPC, and state estimators
        
        function obj = runSimulation(obj, numSteps, r, observer, controller)
            % Main control loop for simulation mode
            % This integrates models
            
            fprintf('\n Project 4: Digital Twin Simulation ===\n');
            fprintf('\n');
            fprintf('  Steps: %d\n', numSteps);
            fprintf('  Ts:    %.3f s\n', obj.Ts);
            fprintf('  RLS λ: %.2f\n', obj.lambda_rls);
            fprintf('  MPC Np: %d, Nc: %d\n', obj.Np, obj.Nc);
            fprintf('\n\n');
            fprintf('Progress: ');
            
            for k = 1:numSteps
                % Progress indicator
                if mod(k, floor(numSteps/20)) == 0
                    fprintf('.');
                end
                
                % Get reference for current step
                if k <= size(r, 2)
                    r_k = r(:, k);
                else
                    r_k = r(:, end);
                end
                
                % Get measurement from observer (Project 2)
                if nargin >= 4 && ~isempty(observer)
                    y = observer.getEstimate();
                else
                    y = obj.x;  % Use true state if no observer
                end
                
                % Compute control using controller (Project 1) or MPC
                if nargin >= 5 && ~isempty(controller)
                    u = controller.compute(r_k, y);
                else
                    u = obj.mpcCompute(obj.x, r_k);  % Use MPC
                end
                
                % Update Digital Twin (RLS Adaptation)
                x_pred = obj.predictState(obj.x, u, obj.p_hat);
                [obj, p_new, ~] = obj.rlsUpdate(obj.x, u, y);
                obj.p_hat = p_new;
                
                % Update Digital Twin state
                obj.x = obj.predictState(obj.x, u, obj.p_hat);
                
                % Store history
                obj.x_history = [obj.x_history, obj.x];
                obj.u_history = [obj.u_history, u];
                obj.p_history = [obj.p_history, p_new];
                obj.t_current = obj.t_current + obj.Ts;
            end
            
            fprintf('\n\n\n');
            fprintf('  Simulation Completed!\n');
            fprintf('  Final Parameters:\n');
            fprintf('    Damping:     %.4f\n', obj.p_hat(1));
            fprintf('    Thrust Gain: %.4f\n', obj.p_hat(2));
            fprintf('\n');
        end
        
        %.............................................................
        % Plotting Results
        % to validate the model accuracy
        % to reveal oscillations, overshoot, and settling time
        
        function plotResults(obj)
            figure('Position', [100, 100, 1200, 600]);
            
            % Subplot 1: Theta (Angle)
            subplot(2,3,1);
            plot(obj.x_history(1,:), 'b-', 'LineWidth', 1.5);
            title('\theta (Angle)', 'FontSize', 12);
            xlabel('Time Step'); ylabel('rad');
            grid on;
            
            % Subplot 2: Omega (Velocity)
            subplot(2,3,2);
            plot(obj.x_history(2,:), 'r-', 'LineWidth', 1.5);
            title('\omega (Angular Velocity)', 'FontSize', 12);
            xlabel('Time Step'); ylabel('rad/s');
            grid on;
            
            % Subplot 3: Adapted Parameters (RLS)
            subplot(2,3,3);
            plot(obj.p_history(1,:), 'g-', 'LineWidth', 1.5); hold on;
            plot(obj.p_history(2,:), 'm-', 'LineWidth', 1.5);
            plot([1, length(obj.p_history)], [obj.p(1), obj.p(1)], ...
                'g--', 'LineWidth', 1);
            plot([1, length(obj.p_history)], [obj.p(2), obj.p(2)], ...
                'm--', 'LineWidth', 1);
            title('Parameter Adaptation (RLS)', 'FontSize', 12);
            xlabel('Time Step'); ylabel('Value');
            legend('Damping (est)', 'Thrust Gain (est)', ...
                   'Damping (true)', 'Thrust Gain (true)', ...
                   'Location', 'best');
            grid on;
            
            % Subplot 4: Control Input
            subplot(2,3,4);
            plot(obj.u_history, 'k-', 'LineWidth', 1.5);
            title('Control Input', 'FontSize', 12);
            xlabel('Time Step'); ylabel('u');
            grid on;
            
            % Subplot 5: Phase Portrait
            subplot(2,3,5);
            plot(obj.x_history(1,:), obj.x_history(2,:), 'b-', 'LineWidth', 1);
            hold on;
            plot(obj.x_history(1,1), obj.x_history(2,1), 'go', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot(obj.x_history(1,end), obj.x_history(2,end), 'ro', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'r');
            title('Phase Portrait', 'FontSize', 12);
            xlabel('\theta (rad)'); ylabel('\omega (rad/s)');
            legend('Trajectory', 'Start', 'End', 'Location', 'best');
            grid on;
            
            % Subplot 6: Parameter Estimation Error
            subplot(2,3,6);
            damping_error = obj.p_history(1,:) - obj.p(1);
            thrust_error = obj.p_history(2,:) - obj.p(2);
            plot(damping_error, 'g-', 'LineWidth', 1.5); hold on;
            plot(thrust_error, 'm-', 'LineWidth', 1.5);
            plot([1, length(obj.p_history)], [0, 0], 'k--', 'LineWidth', 1);
            title('Parameter Estimation Error', 'FontSize', 12);
            xlabel('Time Step'); ylabel('Error');
            legend('Damping Error', 'Thrust Gain Error', 'Location', 'best');
            grid on;
            
            sgtitle('Project 4: Digital Twin Simulation Results', ...
                'FontSize', 14, 'FontWeight', 'bold');
        end
        %.............................................................
        % Export Data
        % to write workspace variables for offline analysis
        
        function exportData(obj, filename)
            % EXPORTDATA Export simulation data to MAT file
            
            if nargin < 2
                filename = 'digital_twin_data.mat';
            end
            
            data.Ts = obj.Ts;
            data.t_current = obj.t_current;
            data.x_history = obj.x_history;
            data.u_history = obj.u_history;
            data.p_history = obj.p_history;
            data.p_true = obj.p;
            data.p_final = obj.p_hat;
            data.lambda_rls = obj.lambda_rls;
            data.Np = obj.Np;
            data.Nc = obj.Nc;
            
            save(filename, 'data');
            fprintf('Data exported to: %s\n', filename);
        end
        
        %.............................................................
        % Reset Simulation
        % to reinitialize the parameters for next start
       
        function obj = reset(obj, x0, p0)
            % RESET Reset the digital twin to initial conditions
            
            if nargin < 2
                x0 = [0; 0];
            end
            if nargin < 3
                p0 = obj.p_hat;
            end
            
            obj.x = x0;
            obj.x_history = x0;
            obj.u_history = 0;
            obj.p_history = p0;
            obj.p_hat = p0;
            obj.theta_rls = p0;
            obj.t_current = 0;
        end
        
        %.............................................................
        % Display Object Information
        % to update numerical readouts and warning flags during runtime
      
        function disp(obj)
            % DISP Display digital twin information
            
            fprintf('Different Object (Digital Twin)\n');
            fprintf('\n');
            fprintf('  Sampling Time (Ts): %.3f s\n', obj.Ts);
            fprintf('  Current Time:       %.3f s\n', obj.t_current);
            fprintf('  State (x):          [%.3f, %.3f]\n', obj.x(1), obj.x(2));
            fprintf('  Parameters (p_hat): [%.3f, %.3f]\n', obj.p_hat(1), obj.p_hat(2));
            fprintf('  RLS Lambda:         %.3f\n', obj.lambda_rls);
            fprintf('  MPC Np/Nc:          %d/%d\n', obj.Np, obj.Nc);
            fprintf('\n');
        end
    end
end
