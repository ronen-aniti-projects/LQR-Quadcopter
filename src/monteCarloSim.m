%% Monte Carlo Simulation 
% In this file, I implment a Monte Carlo simulation as a means of answering
% whether or not the LQR controller I derived in previous modules actually
% does well to stabilize the simulated quadcopter dynamics when the
% simulated quadcopter dynamics are given some initial value offset from
% the equilibrium state. 

% Define the number of trials
numTrials = 1500;

% Define the time step for numerical integration (Euler)
dt = 0.01;

% Define the total simulation runtime for each trial
Tfinal = 10.0;

% Define a tolerance for determining settling
tol = 0.02;

% Define the minimum required time before deeming a state as being settled
holdTime = 0.10;

% Seed the random number generator
rng(42);

% Define the bounds of the initial condition windows on each state
initWindows = [ ...
    -1.0 1.0; % x
    -1.0 1.0; % y
    -1.0 1.0; % z
    -1.0 1.0;
    -1.0 1.0;
    -1.0 1.0;
    0.0 0.0;
    0.0 0.0;
    0.0 0.0;
    0.0 0.0;
    0.0 0.0;
    0.0 0.0];

% Load quadcopter constants from a prior module
m = parameterValues(1);
g = parameterValues(2);

% Define the required thrust to maintain only hover
Fc_hover = m*g;

% Define generic Q and R matrices
Q = eye(12);
R = eye(4);

% Form a control law gain matrix
K = lqr(AHoverEvaluated, BHoverEvaluated, Q, R);

% Define the equilibrium state (desired)
xDes = zeros(12,1);

% Define the hover thrust vector
hoverThrust = [Fc_hover;0;0;0];

% Define the number of simulation steps
N = floor(Tfinal/dt);

% Form a time vector
t = (0:N)*dt;

% Define how many time steps sufficient for settling
holdN = round(holdTime/dt);

% Define storage for each trial's settling time
settleTimes = zeros(numTrials,1);

% Define storage for whether/not each trial settles
didSettle = zeros(numTrials,1);

% Define storage for each trial's thrust overhead
% i.e. % max thrust is over hover thrust
thrustOverhead = zeros(numTrials,1);

% Define storage for each trial's percent overshoot
overshoot = zeros(numTrials,1);

% Iterate trials
for trial = 1:numTrials
    % Generate a random initial condition state
    initialState = zeros(12,1);
    for stateIndex = 1:12
        a = initWindows(stateIndex,1);
        b = initWindows(stateIndex,2);
        initialState(stateIndex) = a + (b - a) * rand;
    end
    
    % Assign the initial state to the current state
    x = initialState;
    
    % Record the signs of each state now so that later, we can determine if
    % the equilibrium point was crossed.
    initialSigns = sign(initialState);

    % Storage for the maximum value achieved on each state once any sign
    % change occurs. Defaults to zero on each state. 
    maxOppositeDeflection = zeros(12,1);
    
    % Counter for how many time steps all states have settled
    settleCounter = 0;

    % Counter for the seconds duration that all states have settled
    settleTime = 0;

    % Track maximum thrust, with hover thrust being an initial value
    maxThrustCommand = Fc_hover;
    
    % Begin the control loop simulation
    for step = 1:N

        % Generate the control vector for the given state
        u = -K*(x - xDes) + hoverThrust;

        % Compute the state derivative vector based on the nonlinear
        % quadcopter dynamics model
        xdot = evaluateDynamics(parameterValues, x, u);
        
        % Integrate the state forward in time by one step with Euler
        % integration
        x = x + dt*xdot;
        
        % If the collective thrust is greater than the value recorded for
        % maximum thrust for this trial, then replace the recorded value
        % with the value at this time step. This track's the trial's
        % maximum collective thrust. 
        if u(1) > maxThrustCommand
            maxThrustCommand = u(1);
        end

        % For all 12 states, track the maximum opposite deflection. Use the
        % maximum of the maximum opposite deflections to compute overshoot
        % once the trial time has finished. 
        for stateIndex = 1:12
            if initialSigns(stateIndex) ~= 0
                if sign(x(stateIndex)) == -initialSigns(stateIndex)
                    deflection = abs(x(stateIndex));
                    if deflection > maxOppositeDeflection(stateIndex)
                        maxOppositeDeflection(stateIndex) = deflection;
                    end
                end
            end
        end
    
        % Check settling. Settling occurs only when all states are within
        % tolerance for the hold time. If settling occurs, record how long 
        % it takes to settle.
        if all(abs(x - xDes) <= tol)
            settleCounter = settleCounter + 1;
            if settleCounter >= (holdN+1)
                settleTime = t(step - holdN);
                break
            end
        else
            settleCounter = 0;
        end
    % End of the trial.
    end 
    
    % If settling did occur, record it in storage.
    if settleTime > 0
        settleTimes(trial) = settleTime;
        didSettle(trial) = 1;
    end
    
    % Compute "Thrust Overhead" as the percent difference between the
    % maximum trial thrust and the hover thrust. 
    thrustOverhead(trial) = 100 * (maxThrustCommand - Fc_hover) / Fc_hover;

    % Compute overshoot: I edited this portion recently. I redefine
    % overshoot as simply an absolute value of the maximum deflection on
    % only positional states. (Not across all states anymore).
    overshootEachState = zeros(12,1);
    for stateIndex = 1:3 % Changed to only measure states 1 to 3
        initialMagnitude = abs(initialState(stateIndex));
        if initialMagnitude > 0
            overshootEachState(stateIndex) = maxOppositeDeflection(stateIndex);
        else
            % Prevent division by zero
            overshootEachState(stateIndex) = 0;
        end
    end
    overshoot(trial) = max(overshootEachState);

end

% Extract all trial numbers that settled
settledIndices = find(didSettle==1);

% Calculate the median settle time across all trials
medianSettle = median(settleTimes(settledIndices));

% Calculate the mean settling time across all trials
meanSettle = mean(settleTimes(settledIndices));

% Configure histogram bins (30 bins)
edgesSettleTime = linspace(0, Tfinal, 31);

% Plot the results
figure;

sgtitle('Monte Carlo Evaluation of Quadcopter LQR Hover Controller');

subplot(1,3,1);
histogram(settleTimes(settledIndices), edgesSettleTime, 'FaceColor', [0.2 0.6 0.9]);
title('Settling Time');
xlabel('Time [s]'); ylabel('Count'); grid on; box on;

subplot(1,3,2);
histogram(thrustOverhead, 30, 'FaceColor', [0.2 0.8 0.2]); 
title('Thrust Overhead (%)');
xlabel('Overhead [%]'); ylabel('Count'); grid on; box on;

subplot(1,3,3);
histogram(overshoot, 30, 'FaceColor', [0.9 0.3 0.3]); 
title('Overshoot (m)');
xlabel('Overshoot [m]'); ylabel('Count'); grid on; box on;

% Show simulation statistics (mean settling, median settling, % settled)
fprintf('Settled: %d / %d (%.1f%%)\n', sum(didSettle), numTrials, 100 * sum(didSettle) / numTrials);
fprintf('Median Settling Time: %.3f s\n', medianSettle);
fprintf('Mean Settling Time: %.3f s\n', meanSettle);

%% calculate important stats

% Extract indices of settled trials
settledIndices = find(didSettle==1);

% Compute the percent of trials that settle
pctSettled = 100 * numel(settledIndices) / numTrials;

% Settling Time Statistics
STMean = mean(settleTimes(settledIndices));
STMedian = median(settleTimes(settledIndices));
STP95 = prctile(settleTimes(settledIndices), 95);

% Thrust Overhead Statistics
thrustOverheadMean= mean(thrustOverhead);
thrustOverheadMedian = median(thrustOverhead);
thrustOverheadP95 = prctile(thrustOverhead, 95);

% Overshoot Statistics
overshootMean = mean(overshoot);
overshootMedian = median(overshoot);
overshootP95 = prctile(overshoot, 95);

% Print Results
fprintf('\n--- Monte Carlo Simulation Results Summary ---\n');
fprintf('Settled: %d / %d (%.1f%%)\n', numel(settledIndices), numTrials, pctSettled);
fprintf('Settling Time [s]: mean=%.3f  median=%.3f  p95=%.3f\n', STMean, STMedian, STP95);
fprintf('Thrust Overhead [%%]: mean=%.2f  median=%.2f  p95=%.2f\n', thrustOverheadMean, thrustOverheadMedian, thrustOverheadP95);
fprintf('Overshoot [m]: mean=%.3f  median=%.3f  p95=%.3f\n', overshootMean, overshootMedian, overshootP95);
