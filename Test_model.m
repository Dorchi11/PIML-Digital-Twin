%% Test the model

YPred = predict(net, XTestN) .* sigY + muY;    % predict new data
rmse = sqrt(mean((YPred - YTest).^2, 1));

%% Evaluate

rmse = sqrt(mean((YPred - YTest).^2, 1));   % compare predicted value with real value
mae  = mean(abs(YPred - YTest), 1);
R2   = 1 - sum((YTest - YPred).^2) ./ sum((YTest - mean(YTest,1)).^2);

%% Print results

fprintf('\n Test Performance:\n');
fprintf('deltaTheta : RMSE = %.6e | MAE = %.6e | R2 = %.4f\n', ...
    rmse(1), mae(1), R2(1));
fprintf('deltaOmega : RMSE = %.6e | MAE = %.6e | R2 = %.4f\n', ...
    rmse(2), mae(2), R2(2));

%% Plot deita_theta

figure;
plot(1:length(YTest(:,1)), YTest(:,1), 'LineWidth', 1.0);
hold on;
plot(1:length(YPred(:,1)), YPred(:,1), '--', 'LineWidth', 1.0);
grid on;

xlabel('Sample');
ylabel('\Delta\theta (rad)');
title('PIML NN — Delta Theta');
legend('\Delta\theta_{actual}', '\Delta\theta_{pred}');

%% Plot delta_omega

figure;
plot(1:length(YTest(:,2)), YTest(:,2), 'LineWidth', 1.0);
hold on;
plot(1:length(YPred(:,2)), YPred(:,2), '--', 'LineWidth', 1.0);
grid on;

xlabel('Sample');
ylabel('\Delta\omega (rad/s)');
title('PIML NN — Delta Omega');
legend('\Delta\omega_{actual}', '\Delta\omega_{pred}');

%% DISPLAY

disp('Simulation complete.');
