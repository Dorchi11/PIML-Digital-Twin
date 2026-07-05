clc;
clear;
close all;
%.............................................................
% to design the algorithms for RLS and MPC
%...........................................................
% time parameters
        Ts = 0.01;  % Sampling time
        t_current = 10;  % Current simulation time
        
        % state parameters
        x = [0; 0];          % [theta; omega]
        x_history = zeros(1,N_steps);   % Store states
        u_history = zeros(1,N_steps);   % Store control inputs
        
        % Parameters (adapted by RLS) 
        p.damping = 0.5;  % inital guass
        p.thrust_gain = 1.2;  % initial guass
        p           % [damping; thrust_gain]
        p_hat       % Estimated parameters
        p_history   % Store parameter history
        
        % RLS algorithm
        % to estimate the output
        % to compute the error
        % to update the variables/parameters
        theta_rls = [p.damping; p.thrust_gain];   % RLS parameter vector
        P_rls = 1000*eye(2);       % RLS covariance matrix
        lambda_rls = 0.98;  % Forgetting factor (0.95-0.99)
        
        % MPC algorithm
        % to predict the future states
        % to select the optimal control input signal
        % to balance the target
        Np = 10;          % Prediction horizon
        Nc          % Control horizon
        Q           % State weighting matrix
        R           % Input weighting matrix
        u_min       % Min control input
        u_max       % Max control input

       % Start the function
       % to convert mathematical models into real-time actions
       % to set all variables
        function obj = DigitalTwin(Ts, x0, p0)
            obj.Ts = Ts;
            obj.x = x0;
            obj.p = p0;
            obj.p_hat = p0;
            
            % Initialize RLS
            obj.theta_rls = p0;
            obj.P_rls = 1e3 * eye(2);
            obj.lambda_rls = 0.98;
            
            % Initialize MPC
            obj.Np = 10;
            obj.Nc = 3;
            obj.Q = diag([10, 1]);
            obj.R = 0.1;
            obj.u_min = -20;
            obj.u_max = 20;
            
            % Initialize history
            obj.x_history = x0;
            obj.u_history = 0;
            obj.p_history = p0;
        end
        
        %..............................................................
        % State Prediction physical model
        % to predict and update the time using mathematical model and current inputs
        % to calculate the difference between actual measurement and predicted output using new sensor data
        
            x_next = x+Ts*f(x, u, p);
            % Uses the hybrid thrust model from Project 3
            % This is the physics-informed core of the digital twin
            
            % Unpack states
            theta = x_current(1);
            omega = x_current(2);
            
            % Unpack parameters
            damping = p(1);
            thrust_gain = p(2);
            
            % --- Compute thrust using Project 3's hybrid model ---
            % This will call hybridThrustModel function
            thrust = obj.hybridThrustModel(u, thrust_gain);
            
            % Moment of inertia (from your system)
            J = 0.01;
            
            % Dynamics: theta_dot = omega
            %          omega_dot = (1/J) * (thrust - damping * omega)
            theta_dot = omega;
            omega_dot = (1/J) * (thrust - damping * omega);
            
            % Euler integration
            theta_next = theta + obj.Ts * theta_dot;
            omega_next = omega + obj.Ts * omega_dot;
            
            x_next = [theta_next; omega_next];
        end
        
        %...............................................................
        % RLS Parameter Adaptation function
        % to compute the adaptive gain vector
        % to apply the forgetting factor
 
        [theta_new, P_new] = rls_Update(phi, y, theta, p, lamda);
            % Recursive Least Squares for online parameter adaptation
            %
            % Inputs:
            %   x_current  - Current predicted state
            %   u          - Control input
            %   x_measured - Measured state (from Project 2)
            % Outputs:
            %   p_new      - Updated parameters [damping; thrust_gain]
            
            % Sensitivity matrix (Jacobian w.r.t parameters)
            omega = x_current(2);
            J = 0.01;
            
            % psi = [∂omega_dot/∂damping, ∂omega_dot/∂thrust_gain]
            psi = zeros(2, 2);
            psi(2, 1) = -omega / J;     % Sensitivity to damping
            psi(2, 2) = u^2 / J;        % Sensitivity to thrust_gain
            
            % Prediction error (innovation)
            error = x_measured - x_current;
            
            % Kalman gain for RLS
            K = obj.P_rls * psi' / (psi * obj.P_rls * psi' + 1);
            
            % Update parameters
            theta_new = obj.theta_rls + K * error;
            
            % Update covariance (with forgetting factor)
            P_new = (1/obj.lambda_rls) * (obj.P_rls - K * psi * obj.P_rls);
            
            % Store for next iteration
            obj.theta_rls = theta_new;
            obj.P_rls = P_new;
            obj.p_hat = theta_new;
            
            p_new = theta_new;
        end
        
        %................................................................
        % Model Predictive Control function
        % to calculate the optimal future voltage/current sequence
        % to apply the constraints
       
       u = mpc_Compute(obj, x0, r)
            % Model Predictive Control - computes optimal control input
            %
            % Inputs:
            %   x0 - Current state
            %   r  - Reference trajectory
            % Output:
            %   u_opt - Optimal control input
            
            % Initial guess for control sequence
            u0 = zeros(obj.Nc, 1);
            
            % Bounds
            lb = obj.u_min * ones(obj.Nc, 1);
            ub = obj.u_max * ones(obj.Nc, 1);
            
            % Solve optimization using fmincon
            options = optimoptions('fmincon', ...
                'Display', 'off', ...
                'Algorithm', 'sqp', ...
                'MaxIterations', 100);
            
            [u_opt, ~] = fmincon(@(u) obj.mpcCost(x0, u, r), ...
                u0, [], [], [], [], lb, ub, ...
                @(u) obj.mpcConstraints(x0, u), options);
            
            % Return only the first control input
            u_opt = u_opt(1);
        end
        
        %% MPC Cost Function
        function J = mpcCost(obj, x0, u_seq, r)
            J = 0;
            x = x0;
            
            for k = 1:obj.Np
                if k <= obj.Nc
                    u = u_seq(k);
                else
                    u = u_seq(end);
                end
                
                x = obj.predictState(x, u, obj.p_hat);
                e = x - r(:, k);
                J = J + e' * obj.Q * e + obj.R * u^2;
            end
        end
        
        % to define the constraints for mpc
        
        function [c, ceq] = mpcConstraints(~, ~, ~)
            c = [];
            ceq = [];
        end
        
        %............................................................
        % interface: Call Project 3's Hybrid Thrust Model
       
        function thrust = hybridThrustModel(obj, u, thrust_gain)
            % This is an interface to Project 3's model
            % The actual implementation will be in hybridThrustModel.m
            
            % Simple physics model (placeholder)
            % In reality, this will call the actual Project 3 function
            thrust = thrust_gain * u^2;
            
            % Uncomment when Project 3 is ready:
            % params.thrust_gain = thrust_gain;
            % thrust = hybridThrustModel(u, params);
        end
        
        %...............................................................
        % Main control loop 
        % to ensure prediction,optimisations, and parameter updates
        % to allow RLS, MPC, amd estate estimators
       
        function runSimulation(obj, numSteps, r_trajectory, observer, controller)
            % Main control loop for simulation mode
            % This uses Project 1 (controller) and Project 2 (observer)
            %
            % Inputs:
            %   numSteps     - Number of simulation steps
            %   r_trajectory - Reference trajectory
            %   observer     - Project 2: EKF observer
            %   controller   - Project 1: PID controller
            
            fprintf('\n=== Project 4: Digital Twin Simulation ===\n');
            
            for k = 1:numSteps
                % --- Get reference ---
                r = r_trajectory(:, k);
                
                % --- Get measurement from Project 2 (observer) ---
                y = observer.getEstimate();
                
                % --- Compute control using Project 1 (controller) ---
                u = controller.compute(r, y);
                
                % --- Update Digital Twin (RLS Adaptation) ---
                x_pred = obj.predictState(obj.x, u, obj.p_hat);
                [p_new, ~, ~] = obj.rlsUpdate(x_pred, u, y);
                obj.p_hat = p_new;
                
                % --- MPC (optional - can use instead of PID) ---
                % u_mpc = obj.mpcCompute(obj.x, r_trajectory(:, k:k+obj.Np));
                
                % --- Update Digital Twin state ---
                obj.x = obj.predictState(obj.x, u, obj.p_hat);
                
                % --- Store history ---
                obj.x_history = [obj.x_history, obj.x];
                obj.u_history = [obj.u_history, u];
                obj.p_history = [obj.p_history, p_new];
                obj.t_current = obj.t_current + obj.Ts;
            end
            
            fprintf('Simulation Completed!\n');
        end
        
        %...........................................................
        % plottingt
        % to validate the model accuracy
        % to reveal oscillations, overshoot, and settling time
       
        function plotResults(obj)
            figure('Position', [100, 100, 1200, 600]);
            
            % States parameters
            subplot(2,3,1);
            plot(obj.x_history(1,:), 'b-', 'LineWidth', 1.5);
            title('Theta (Angle)'); xlabel('Step'); ylabel('rad');
            grid on;
            
            subplot(2,3,2);
            plot(obj.x_history(2,:), 'r-', 'LineWidth', 1.5);
            title('Omega (Velocity)'); xlabel('Step'); ylabel('rad/s');
            grid on;
            
            % Adapted parameters
            subplot(2,3,3);
            plot(obj.p_history(1,:), 'g-', 'LineWidth', 1.5); hold on;
            plot(obj.p_history(2,:), 'm-', 'LineWidth', 1.5);
            title('Adapted Parameters (RLS)');
            xlabel('Step'); ylabel('Value');
            legend('Damping', 'Thrust Gain');
            grid on;
            
            % Control input parameters
            subplot(2,3,4);
            plot(obj.u_history, 'k-', 'LineWidth', 1.5);
            title('Control Input');
            xlabel('Step'); ylabel('u');
            grid on;
            
            sgtitle('Project 4: Digital Twin Results');
        end
      

 
