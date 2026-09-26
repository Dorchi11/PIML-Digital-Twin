clear; 
clc; 
close all;

%% Train PIML
% Read Data
data = readtable('dataset.csv');

%% Input Features
X = [data.U_real, data.Theta_phy, data.Omega_phy];

deltaTheta = data.Theta_real - data.Theta_phy;
deltaOmega = data.Omega_real - data.Omega_phy;

%% Target Output
Y = [deltaTheta, deltaOmega];

%% Train NN model 

n = size(X,1); rng(42); idx = randperm(n);      % spliting data size
nTrain = round(0.70*n); nVal = round(0.15*n);
tr = idx(1:nTrain); va = idx(nTrain+1:nTrain+nVal); te = idx(nTrain+nVal+1:end);
XTrain=X(tr,:); YTrain=Y(tr,:);
XVal=X(va,:);   YVal=Y(va,:);
XTest=X(te,:);  YTest=Y(te,:);

%% Normalise

muX=mean(XTrain,1); sigX=std(XTrain,1); sigX(sigX==0)=1; % scalling at same size
muY=mean(YTrain,1); sigY=std(YTrain,1); sigY(sigY==0)=1;
XTrainN=(XTrain-muX)./sigX; XValN=(XVal-muX)./sigX; XTestN=(XTest-muX)./sigX;
YTrainN=(YTrain-muY)./sigY; YValN=(YVal-muY)./sigY; YTestN=(YTest-muY)./sigY;

%% networks

layers = [                           % prepare NN layer architecture
    featureInputLayer(3)
    fullyConnectedLayer(64); batchNormalizationLayer; reluLayer; dropoutLayer(0.1)
    fullyConnectedLayer(64); batchNormalizationLayer; reluLayer; dropoutLayer(0.1)
    fullyConnectedLayer(32); reluLayer
    fullyConnectedLayer(2)
];

%% options

options = trainingOptions('adam', ...     % training architecture
    'InitialLearnRate', 1e-3, ...
    'MaxEpochs', 150, ...
    'MiniBatchSize', 256, ...
    'Shuffle','every-epoch', ...
    'ValidationData', {XValN, YValN}, ...
    'ValidationFrequency', 30, ...
    'ValidationPatience', 20, ...
    'Plots','training-progress', ...
    'Verbose', false);

%% train

net = trainnet(XTrainN, YTrainN, layers, "mse", options); % update weight and bias

%% Save the model

save('piml_residual_model.mat','net','muX','sigX','muY','sigY');


