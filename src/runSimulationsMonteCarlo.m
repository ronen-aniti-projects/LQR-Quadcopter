clear; clc; close all;

% Load the linear model
load('QuadcopterModel.mat','AHoverEvaluated','BHoverEvaluated',...
     'measurementMatrix', 'mValue','gValue','IxValue','IyValue','IzValue');

% Load the linear model
load('LQRGains.mat','KData');

% Prepare the parameterValues array for evaluateDynamics
parameterValues = [mValue, gValue, IxValue, IyValue, IzValue];


% ==============================================================================
% ===== SIMULATION 2: Performance Evaluation of Controller 4 ==========
% ==============================================================================

% Monte Carlo Simulation with Improved Metrics
numTrials = 100;
overshootResults = zeros(numTrials, 1);
settlingTimeResults = zeros(numTrials, 1);
thrustOverheadResults = zeros(numTrials, 1);

% Absolute tolerances [m, rad, m/s, rad/s] (modify as needed)
positionTol = 0.02;    % 2 cm
angleTol = 0.0349;     % 2 degrees
velocityTol = 0.1;     % 10 cm/s
rateTol = 0.1;         % 0.1 rad/s (~5.7 deg/s)

% Tolerance vector matching state order: 
% [x,y,z, phi,theta,psi, xd,yd,zd, p,q,r]
tolerances = [positionTol*ones(3,1);
              angleTol*ones(3,1);
              velocityTol*ones(3,1);
              rateTol*ones(3,1)];

% Controller parameters
aggressiveK = KData{4};
hoverThrust = mValue * gValue;
perturbationMagnitude = 0.25;  % +/- 25% from equilibrium

for trial = 1:numTrials
    % 1. Inituial pertubation )
    xPert = perturbationMagnitude * (2*rand(12,1) - 1);
    
    % 2. Simulate dynamics
    dt = 0.01;
    t = 0:dt:20-dt;
    x = zeros(12, length(t));
    u = zeros(4, length(t));
    x(:,1) = xPert;
    
    for ii = 1:length(t)-1
        u(:,ii) = -aggressiveK * x(:,ii) + [hoverThrust; 0; 0; 0];
        xDot = evaluateDynamics(parameterValues, x(:,ii), u(:,ii));
        x(:,ii+1) = x(:,ii) + dt * xDot;
    end
    
    % 3. Compute metrics 
    % Overshoot: Max absolute deviation from equilibrium (0)
    overshootResults(trial) = max(abs(x(1:3)));
    
    % Settling time: First time all states within tolerances
    settled = false(1, length(t));
    for jj = 1:length(t)
        settled(jj) = all(abs(x(:,jj)) <= tolerances);
    end
    firstSettled = find(settled, 1, 'first');
    if ~isempty(firstSettled)
        settlingTimeResults(trial) = t(firstSettled);
    else
        settlingTimeResults(trial) = t(end); % Mark as never settled
    end
    
    % Thrust overhead <- What percent over hover thrust (mg) is the max thrust?
    thrustOverheadResults(trial) = (max(u(1,:))/hoverThrust - 1) * 100;
end

% Extract the key performance metrics and print them to the console
fprintf('Control Performance Summary');
fprintf('Trials: %d | Perturbation: ±%.0f%%\n', numTrials, perturbationMagnitude*100);
fprintf('Settling Criteria: %.2fm, %.1f°, %.2fm/s, %.1f°/s\n',...
    positionTol, rad2deg(angleTol), velocityTol, rad2deg(rateTol));
fprintf('Overshoot (95th %%ile): %.3f units\n', prctile(overshootResults, 95));
fprintf('Settling Time (95th %%ile): %.1fs\n', prctile(settlingTimeResults, 95));
fprintf('Max Thrust Overhead: %.1f%%\n\n', max(thrustOverheadResults));

% Plotting the results of the Monte Carlo simulations
figure('Position', [100 100 1200 400]);

subplot(1,3,1);
histogram(overshootResults, 'FaceColor', [0.2 0.6 0.8]);
title('Overshoot Distribution');
xlabel('Max Deviation from Hover (units)');

subplot(1,3,2);
histogram(settlingTimeResults, 'BinEdges', 0:0.5:20, 'FaceColor', [0.4 0.8 0.4]);
title('Settling Time Distribution');
xlabel('Time (s)');

subplot(1,3,3);
histogram(thrustOverheadResults, 'FaceColor', [0.9 0.4 0.2]);
title('Thrust Overhead');
xlabel('Percentage Over Hover Thrust');

% Save Results
save('Revised_MonteCarlo_Results.mat', 'overshootResults', 'settlingTimeResults', 'thrustOverheadResults');