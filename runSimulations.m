clear; clc; close all;

% Load the linear model
load('QuadcopterModel.mat','AHoverEvaluated','BHoverEvaluated',...
     'measurementMatrix', 'mValue','gValue','IxValue','IyValue','IzValue');

% Prepare the parameterValues array for evaluateDynamics
parameterValues = [mValue, gValue, IxValue, IyValue, IzValue];

% ==============================================================================
% ===== SIMULATION 1: EFFECTS OF VARYING LQR GAINS (4 PARAMETER SETS) ==========
% ==============================================================================

% Conduct 4 Simulations with 4 Sets of Parameters for Q and R
QAll = [diag(10*ones(1,12));
        diag(10*ones(1,12));
        diag(ones(1,12));
        diag(ones(1,12))];

RAll = [diag(ones(1,4));
        diag(10*ones(1,4));
        diag(.1*ones(1,4));
        diag(ones(1,4))];

% Store simulation data in cell arrays
eigenvaluesData = cell(4, 1);
KData = cell(4, 1);         
stateHistory = cell(4, 1);
controlHistory = cell(4, 1); 
settlingTimes = zeros(4, 1); 
maxControlEffort = zeros(4, 1);
maxControlEffortFc = zeros(4, 1); % For collective thrust
maxControlEffortMx = zeros(4, 1); % For moment Mx
maxControlEffortMy = zeros(4, 1); % For moment My
maxControlEffortMz = zeros(4, 1); % For moment Mz

for i=1:4

    % LQR cost matrices
    Q = QAll(12*(i-1)+1:12*i,:);
    R = RAll(4*(i-1)+1:4*i,:);
    
    % Solve for the LQR full-state feedback gains
    K = lqr(AHoverEvaluated, BHoverEvaluated, Q, R);
    
    % Verify that all eigenvalues of the CL system are stable
    disp("The eigenvalues of the closed loop system")
    eigenvaluesControl = eig(AHoverEvaluated - BHoverEvaluated * K);
    disp(eigenvaluesControl)

    % Save the array of eigenvalues for analysis later
    eigenvaluesData{i} = eigenvaluesControl;

    % Save the K matrix for analysis later
    KData{i} = K;

    % The simulation setup
    dt = 0.01;
    t = 0:dt:20-dt; % The timeframe of the simulation
    u = zeros(4,length(t)); % The control history
    x = zeros(12,length(t)); % The true state history
    y = zeros(size(measurementMatrix,1),length(t)); % The measurement history
    xHat = zeros(12,length(t)); % State estimate history
    xDes = [0;0;0;0;0;0;0;0;0;0;0;0]; % The desired state
    xPert = -0.25*ones(1,12); % Perturbed state
    
    x(:, 1) = xPert;
    
    for ii=1:length(t)-1
        
        % Simulate a measurement
        y(:, ii) = measurementMatrix * x(:, ii);
    
        % Estimate the true state
        xHat(:, ii) = y(:, ii); % Ideal estimation with perfect measurement
    
        % The control law
        u(:, ii) = -K * x(:, ii) + [mValue*gValue;0;0;0];
    
        % Advance the true dynamics
        xDot = evaluateDynamics(parameterValues, x(:,ii), u(:,ii));
        x(:,ii+1) = dt * xDot + x(:, ii);
    
    end
    
    % States 1 to 3: x, y, z
    figure;
    plot(t, x(1, :), t, x(2, :), t, x(3, :))
    title("States 1 to 3: x, y, z")
    
    % States 4 to 6: phi, theta, psi
    figure;
    plot(t, x(4, :), t, x(5, :), t, x(6, :))
    title("States 4 to 6: phi, theta, psi")
    
    % States 7 to 9: xdot, ydot, zdot
    figure;
    plot(t, x(7, :), t, x(8, :), t, x(9, :))
    title("States 7 to 9: xdot, ydot, zdot")
    
    % States 10 to 12: p, q, r
    figure;
    plot(t, x(10, :), t, x(11, :), t, x(12, :))
    title("States 10 to 12: p, q, r")
    
    % Actuation history for applied collective thrust
    figure;
    plot(t, u(1,:))
    title("Actuation History: Collective Thrust")
    
    % Actuation history for applied moments
    figure; 
    plot(t, u(2:4, :));
    title("Actuation History: Commanded Torques")
    
    % Save simulation results
    stateHistory{i} = x;
    controlHistory{i} = u;


    maxStates(i, :) = max(abs(x), [], 2); % Maximum value of each state
    % Max control effort for each input
    maxControlEffortFc(i) = max(abs(u(1, :))); % Max effort for Fc
    maxControlEffortMx(i) = max(abs(u(2, :))); % Max effort for Mx
    maxControlEffortMy(i) = max(abs(u(3, :))); % Max effort for My
    maxControlEffortMz(i) = max(abs(u(4, :))); % Max effort for Mz 
    
    % Find the settling time (last state to settle)
    tolerance = 0.02; % 2% threshold
    stateSettlingTimes = zeros(1, 12);
    for j = 1:12
        settledIndex = find(abs(x(j, :)) <= tolerance, 1); % First index within tolerance
        if ~isempty(settledIndex)
            stateSettlingTimes(j) = t(settledIndex);
        else
            stateSettlingTimes(j) = t(end); % Default to max time if not settled
        end
    end
    settlingTimes(i) = max(stateSettlingTimes); % Time for the last state to settle

end

save('LQR_Analysis_Results.mat', 'maxStates', 'settlingTimes');
disp('Max State Values for each parameter set:');
disp(maxStates);
disp('Settling Times (seconds) for each parameter set:');
disp(settlingTimes);
disp('Max Control Effort for each parameter set:');
disp(maxControlEffort);

% Combine the maximum control efforts into a matrix for visualization
maxControlEffortMatrix = [maxControlEffortFc-mValue*gValue, maxControlEffortMx, maxControlEffortMy, maxControlEffortMz];

% Create a grouped bar plot
figure;
figureHandle = figure; % Save the figure handle
bar(maxControlEffortMatrix);
title('Maximum Control Efforts for Each Input Across Parameter Sets');
ylabel('Max Control Effort');
xlabel('Parameter Sets');
xticks(1:4);
xticklabels({'Set A', 'Set B', 'Set C', 'Set D'});
legend({'Fc (Thrust)', 'Mx (Roll Moment)', 'My (Pitch Moment)', 'Mz (Yaw Moment)'}, ...
        'Location', 'northeast');
grid on;


% Plotting the eigenvalues of the closed-loop system for Q = I, R = I
% Using identity matrices for Q and R
Q_identity = eye(12); % Identity matrix for Q
R_identity = eye(4);  % Identity matrix for R

% Compute the LQR gain matrix
K_identity = lqr(AHoverEvaluated, BHoverEvaluated, Q_identity, R_identity);

% Compute the eigenvalues of the closed-loop system
eigenvalues_closed_loop = eig(AHoverEvaluated - BHoverEvaluated * K_identity);

% Plot the eigenvalues with enhanced visuals
figureHandle = figure; % Save the figure handle
hold on;

% Plot eigenvalues as circles with outlines
plot(real(eigenvalues_closed_loop), imag(eigenvalues_closed_loop), 'o', ...
     'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', 'k', ...
     'MarkerFaceColor', 'w');

% Add grid for clarity
grid on;
grid minor;

% Highlight axes
xline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]); % Highlight real-axis
yline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]); % Highlight imaginary-axis

% Customize the plot
title('Real-Imaginary Plot of Closed-Loop Eigenvalues', ...
      'FontSize', 14, 'FontWeight', 'bold');
xlabel('Real Part', 'FontSize', 12);
ylabel('Imaginary Part', 'FontSize', 12);
set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

% Adjust the axes for clarity
axis equal; % Ensure equal scaling
xlim([-12, 1]); % Adjust x-axis limits for better visibility
ylim([-4, 4]); % Adjust y-axis limits as per the requirement

hold off;

% Save the plot as a PNG file
saveas(figureHandle, 'closed_loop_eigenvalues.png'); % Save as PNG


% Define parameters for the two variants
Q_variant1 = eye(12); % Variant 1: Q is identity
R_variant1 = eye(4);  % Variant 1: R is identity

Q_variant2 = eye(12); % Variant 2: Q is identity
R_variant2 = 0.1 * eye(4); % Variant 2: R is 1/10 identity

% Compute the LQR gain matrices for both variants
K_variant1 = lqr(AHoverEvaluated, BHoverEvaluated, Q_variant1, R_variant1);
K_variant2 = lqr(AHoverEvaluated, BHoverEvaluated, Q_variant2, R_variant2);

% Compute the eigenvalues of the closed-loop systems
eigenvalues_variant1 = eig(AHoverEvaluated - BHoverEvaluated * K_variant1);
eigenvalues_variant2 = eig(AHoverEvaluated - BHoverEvaluated * K_variant2);

% Create the figure for subplots
figure;

% Subplot for Variant 1
subplot(2, 1, 1);
plot(real(eigenvalues_variant1), imag(eigenvalues_variant1), 'o', ...
     'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', 'k', ...
     'MarkerFaceColor', 'w');
grid on;
xline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]);
yline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]);
title('Closed-Loop Eigenvalues: Q = I, R = I', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Real Part', 'FontSize', 10);
ylabel('Imaginary Part', 'FontSize', 10);
set(gca, 'FontSize', 10, 'FontWeight', 'bold');
axis equal;
xlim([-40, 1]);
ylim([-4, 4]);

% Subplot for Variant 2
subplot(2, 1, 2);
plot(real(eigenvalues_variant2), imag(eigenvalues_variant2), 'o', ...
     'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', 'k', ...
     'MarkerFaceColor', 'w');
grid on;
xline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]);
yline(0, '--', 'LineWidth', 1.2, 'Color', [0.5, 0.5, 0.5]);
title('Closed-Loop Eigenvalues: Q = I, R = 0.1*I', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Real Part', 'FontSize', 10);
ylabel('Imaginary Part', 'FontSize', 10);
set(gca, 'FontSize', 10, 'FontWeight', 'bold');
axis equal;
xlim([-40, 1]);
ylim([-4, 4]);

sgtitle('Comparison of Closed-Loop Eigenvalues for Two LQR Configurations', 'FontSize', 14, 'FontWeight', 'bold');

% Save the figure
saveas(gcf, 'closed_loop_eigenvalues_variants.png'); % Save as PNG

% Extract control efforts for trials 4 and 3
controlEffortsTrials = [
    maxControlEffortFc(4)-mValue*gValue, maxControlEffortFc(3)-mValue*gValue;
    maxControlEffortMx(4), maxControlEffortMx(3);
    maxControlEffortMy(4), maxControlEffortMy(3);
    maxControlEffortMz(4), maxControlEffortMz(3)
];

% Labels for inputs
inputLabels = {'Fc', 'Mx', 'My', 'Mz'};

% Create grouped bar plot
figure;
bar(controlEffortsTrials, 'grouped');

% Customize the plot
title('Comparison of Control Efforts', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Control Effort', 'FontSize', 12);
xlabel('Control Inputs', 'FontSize', 12);
set(gca, 'XTickLabel', inputLabels, 'FontSize', 10, 'FontWeight', 'bold');
legend({'Balanced', 'Aggressive'}, 'Location', 'northeast', 'FontSize', 10);
grid on;

% Adjust bar colors for clarity (optional)
colormap([0.2 0.6 0.8; 0.8 0.4 0.2]); % Example color scheme

% Save the figure
saveas(gcf, 'control_efforts_comparison.png'); % Save as PNG

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
    % 1. Initial perturbation (-25% to +25% of max expected values)
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
    
    % 3. Compute metrics ------------------------------------------------
    % Overshoot: Max absolute deviation from equilibrium (0)
    overshootResults(trial) = max(abs(x(1:3,:)));
    
    % Settling time: First time ALL states within tolerances
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
    
    % Thrust overhead
    thrustOverheadResults(trial) = (max(u(1,:))/hoverThrust - 1) * 100;
end

% Resume-Ready Metrics
fprintf('\n=== Control Performance Summary ===\n');
fprintf('Trials: %d | Perturbation: ±%.0f%%\n', numTrials, perturbationMagnitude*100);
fprintf('Settling Criteria: %.2fm, %.1f°, %.2fm/s, %.1f°/s\n',...
    positionTol, rad2deg(angleTol), velocityTol, rad2deg(rateTol));
fprintf('Overshoot (95th %%ile): %.3f units\n', prctile(overshootResults, 95));
fprintf('Settling Time (95th %%ile): %.1fs\n', prctile(settlingTimeResults, 95));
fprintf('Max Thrust Overhead: %.1f%%\n\n', max(thrustOverheadResults));

% Plotting (Original Style)
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