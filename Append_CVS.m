%% main loop
clc
clear
close all

Tsim = 10;             % Simulation time (s)
dt = 0.01;             % Sampling time (s)
x0 = [0; 0];           % Initial conditions [theta(rad), omega(rad/s)]
refSet = -0.5:0.1:0.5;
initSet = -0.5:0.1:0.5;

csvFile = 'dataset.csv';

%% Delete old file
if isfile(csvFile)
    delete(csvFile);
end

%% Write header 
header = {'U_real','Theta_real','Omega_real','Theta_phy', 'Omega_phy'};
writecell(header, csvFile);
simulationNo = 0;

for i = 1:length(refSet)
    for j= 1:length(initSet)
        if refSet(i)~=initSet(j)

            simulationNo = simulationNo+1;
            theta_ref = refSet(i);                  
            x0 = [initSet(j); 0];
             assignin('base', 'x0', x0);
            assignin('base', 'theta_ref', theta_ref);
            simOut = sim('DnominalSys', Tsim);
            disp('Simulation completed successfully.');
            
            %% Extract data
            U_real     = simOut.U_real.Data;
            Theta_real = simOut.Theta_real.Data;
            Omega_real = simOut.Omega_real.Data;
            Theta_phy = simOut.Theta_phy.Data;
            Omega_phy = simOut.Omega_phy.Data;
            
            %% Trim signal lengths
            if size(Theta_phy, 2) > 1
                Theta_phy = Theta_phy(:,1);
            end
            if size(Omega_phy, 2) > 1
                Omega_phy = Omega_phy(:,1);
            end
            
            %% Match lengths
            n = min([length(U_real), length(Theta_real), length(Omega_real), ...
                     length(Theta_phy), length(Omega_phy)]);
            
            U_real     = U_real(1:n);
            Theta_real = Theta_real(1:n);
            Omega_real = Omega_real(1:n);
            Theta_phy = Theta_phy(1:n);
            Omega_phy = Omega_phy(1:n);
            
            %% Create Theta_ref vector to match the length of other signals
            Theta_ref = theta_ref * ones(n, 1);
            
            %% Build table 
            T = table(U_real, Theta_real, Omega_real, Theta_phy, Omega_phy, ...
            'VariableNames', {'U_real','Theta_real','Omega_real','Theta_phy', 'Omega_phy'});
            
            %% Append to CSV
            writetable(T, csvFile,'WriteMode','append','WriteVariableNames',false);
            fprintf("Simulation number: %d \n",simulationNo)
            
        end
    end
end

%% Display results
disp('All runs complete!');
disp(['Saved: ' csvFile]);
winopen(csvFile);
disp('Opening dataset.csv in Excel');
