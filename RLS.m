function [L_hat, b_hat] = rls_estimator(u, theta, omega)
    
    persistent L_hat_p b_hat_p P_rls omega_prev
    
    Ts = 0.01;
    J  = 0.015;
    m  = 0.55;
    lc = 0.13;
    g  = 9.81;
    
    %% Initial guesses 
    L_hat_init = 0.16;
    b_hat_init = 0.010;
    
    %% RLS tuning
    P_rls_init = diag([1000, 100]);
    lambda_min = 0.95;
    lambda_max = 0.999;
    rho        = 500;
    
    %% Bounds
    L_min = 0.10;    L_max = 0.20;
    b_min = 0.005;   b_max = 0.015;
    
    %% Initialise persistent variables 
    if isempty(L_hat_p)
        L_hat_p = L_hat_init;
        b_hat_p = b_hat_init;
        P_rls = P_rls_init;
        omega_prev = omega;
    end
    
    %% Numerical derivative
    theta_ddot_meas = (omega - omega_prev) / Ts;
    
    %% Measurement 
    y = theta_ddot_meas + (m*g*lc/J)*sin(theta);
    
    %% Regressor
    Psi = [ u/J ; -omega/J ];
    
    %% Prediction
    y_hat = Psi' * [L_hat_p; b_hat_p];
    
    %% Prediction error
    e_rls = y - y_hat;
    
    %% Adaptive forgetting factor
    lambda_k = lambda_min + (1 - lambda_min) * exp(-rho * e_rls^2);
    lambda_k = max(lambda_min, min(lambda_k, lambda_max));
    
    %% Kalman gain
    K_gain = (P_rls * Psi) / (Psi' * P_rls * Psi + lambda_k);
    
    %% Parameter update 
    if abs(u) < 9.5
        theta_new = [L_hat_p; b_hat_p] + K_gain * e_rls;
        L_hat_p = max(L_min, min(theta_new(1), L_max));
        b_hat_p = max(b_min, min(theta_new(2), b_max));
        
        %% Covariance update
        P_rls = (P_rls - K_gain * Psi' * P_rls) / lambda_k;
    end
    
    omega_prev = omega;
    
    %% Outputs
    L_hat = L_hat_p;
    b_hat = b_hat_p;
end
