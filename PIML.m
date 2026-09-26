function [deltaTheta, deltaOmega] = fcn(u, theta, omega)

coder.extrinsic('piml_predict');

%% Initialise outputs 
deltaTheta = 0;
deltaOmega = 0;

%% Call the external function with all three inputs
[deltaTheta, deltaOmega] = piml_predict(u, theta, omega);

end
