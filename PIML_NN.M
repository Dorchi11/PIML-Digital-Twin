function [deltaTheta, deltaOmega] = piml_predict(u, theta, omega)
    persistent net muX sigX muY sigY
    
    if isempty(net)
        S = load('piml_residual_model.mat');
        net  = S.net;
        muX  = S.muX;
        sigX = S.sigX;
        muY  = S.muY;
        sigY = S.sigY;
    end
    
    X = [u, theta, omega];
    Xn = (X - muX) ./ sigX;
    Yn = predict(net, Xn);
    Y = Yn .* sigY + muY;
    
    deltaTheta = double(Y(1));
    deltaOmega = double(Y(2));
end
