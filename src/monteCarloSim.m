numTrials = 1500;
dt = 0.01;
Tfinal = 10.0;
tol = 0.02;
holdTime = 0.10;
rng(42);

initWindows = [ ...
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0;
    0.0 1.0];

m = parameterValues(1);
g = parameterValues(2);
Fc_hover = m*g;

Q = eye(12);
R = eye(4);
K = lqr(AHoverEvaluated, BHoverEvaluated, Q, R);

xDes = zeros(12,1);
hoverThrust = [Fc_hover;0;0;0];

N = floor(Tfinal/dt);
t = (0:N)*dt;
holdN = round(holdTime/dt);

settleTimes = zeros(numTrials,1);
didSettle = zeros(numTrials,1);
thrustOverhead = zeros(numTrials,1);
overshootPercent = zeros(numTrials,1);

for trial = 1:numTrials
    % random initial condition
    x = zeros(12,1);
    for stateIndex = 1:12
        a = initWindows(stateIndex,1);
        b = initWindows(stateIndex,2);
        x(stateIndex) = a + (b-a)*rand;
    end

    initialState = x;
    initialSigns = sign(initialState);
    maxOppositeDeflection = zeros(12,1);

    settleCounter = 0;
    settleTime = 0;
    maxThrustCommand = Fc_hover;

    for step = 1:N
        u = -K*(x - xDes) + hoverThrust;
        xdot = evaluateDynamics(parameterValues, x, u);
        x = x + dt*xdot;

        % track max thrust
        if u(1) > maxThrustCommand
            maxThrustCommand = u(1);
        end

        % check overshoot: did it cross past equilibrium in opposite direction?
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

        % check settling
        if all(abs(x - xDes) <= tol)
            settleCounter = settleCounter + 1;
            if settleCounter >= (holdN+1)
                settleTime = t(step - holdN);
                break
            end
        else
            settleCounter = 0;
        end
    end

    if settleTime > 0
        settleTimes(trial) = settleTime;
        didSettle(trial) = 1;
    end

    thrustOverhead(trial) = 100 * (maxThrustCommand - Fc_hover) / Fc_hover;

    % overshoot percentage per state, then take the largest
    overshootEachState = zeros(12,1);
    for stateIndex = 1:12
        initialMagnitude = abs(initialState(stateIndex));
        if initialMagnitude > 0
            overshootEachState(stateIndex) = 100 * maxOppositeDeflection(stateIndex) / initialMagnitude;
        else
            overshootEachState(stateIndex) = 0;
        end
    end
    overshootPercent(trial) = max(overshootEachState);

    % progress print
    if rem(trial,100) == 0
        fprintf('%d of %d done\n', trial, numTrials);
    end
end

settledIndices = find(didSettle==1);
edgesST = linspace(0, Tfinal, 31);

figure;
subplot(1,3,1);
histogram(settleTimes(settledIndices), edgesST, 'FaceColor', [0.2 0.6 0.9]); % blue
title('Settling Time');
xlabel('Time [s]'); ylabel('Count'); grid on; box on;

subplot(1,3,2);
histogram(thrustOverhead, 30, 'FaceColor', [0.2 0.8 0.2]); % green
title('Thrust Overhead (%)');
xlabel('Overhead [%]'); ylabel('Count'); grid on; box on;

subplot(1,3,3);
histogram(overshootPercent, 30, 'FaceColor', [0.9 0.3 0.3]); % red
title('Overshoot (%)');
xlabel('Overshoot [%]'); ylabel('Count'); grid on; box on;

fprintf('Settled: %d / %d (%.1f%%)\n', sum(didSettle), numTrials, 100*sum(didSettle)/numTrials);
fprintf('Median ST: %.3f s\n', medianSettle);
fprintf('Mean   ST: %.3f s\n', meanSettle);
