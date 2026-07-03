%...................................................................
% to test the system in simulation and find the error without hard
%...................................................................

% to setup the digital twin system
Ts = 0.01;
x = [0; 0];
p = [0.1; 0.5];  % [damping; thrust_gain]

dt = DigitalTwin(Ts, x, p);
% get the reference from project 1,2,3

% Project 1: PID Controller (interface)
controller = ControllerPID(10, 0.5, 1, Ts);

% Project 2: EKF Observer (interface)
observer = ObserverEKF(Ts, x);

% Project 3: Hybrid Thrust Model (interface)
% (Already inside DigitalTwin.hybridThrustModel)

% to define reference trajectory
numSteps = 1000;
t = (0:numSteps-1) * Ts;
r_trajectory(1,:) = 0.5 * sin(0.5 * t);
r_trajectory(2,:) = 0.5 * cos(0.5 * t);

% to run simulation
dt.runSimulation(numSteps, r_trajectory, observer, controller);

% to plot the result
dt.plotResults();
