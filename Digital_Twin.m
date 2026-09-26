function [theta_DT, omega_DT] = DT_state(u, theta_DT_prev, omega_DT_prev, L_hat, b_hat)
    
    J  = 0.015;
    m  = 0.55;
    lc = 0.13;
    g  = 9.81;
    Ts = 0.01;
    
    %% Digital Twin dynamics 
    theta_ddot_DT = (L_hat * u ...
                    - m*g*lc*sin(theta_DT_prev) ...
                    - b_hat * omega_DT_prev) / J;
    
    %% Semi-implicit Euler
    omega_DT = omega_DT_prev + theta_ddot_DT * Ts;
    theta_DT = theta_DT_prev + omega_DT * Ts;
end
