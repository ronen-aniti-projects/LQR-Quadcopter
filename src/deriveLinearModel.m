% Symbolic Dynamics Module
syms m g Ix Iy Iz
syms Fc Mx My Mz
syms x xd xdd 
syms y yd ydd
syms z zd zdd
syms phi phid
syms theta thetad
syms psi psid
syms p pd
syms q qd
syms r rd 

% Rotation matrices for ZYX Euler sequence (body to world frame)
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz * Ry * Rx; % Overall rotation matrix

% Angular accelerations (body frame)
pdRHS = Mx/Ix - (Iz-Iy)*r*q/Ix;
qdRHS = My/Iy - (Ix-Iz)*p*r/Iy;
rdRHS = Mz/Iz - (Iy-Ix)*q*p/Iz;

% Linear accelerations (world frame)
linearAccelerations = 1/m * R * [0;0;Fc] - [0;0;g];
xddRHS = linearAccelerations(1);
yddRHS = linearAccelerations(2);
zddRHS = linearAccelerations(3);

% Transform body rates to Euler rates
T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
eulerRates = T * [p;q;r];
phidRHS = eulerRates(1);
thetadRHS = eulerRates(2);
psidRHS = eulerRates(3);

stateSymbolic = [x; y; z; phi; theta; psi; xd; yd; zd; p; q; r];
inputSymbolic = [Fc; Mx; My; Mz];
stateDerivativeSymbolic = [xd; yd; zd; phidRHS; thetadRHS; psidRHS; xddRHS; yddRHS; zddRHS; pdRHS; qdRHS; rdRHS];

% Linearization
A = jacobian(stateDerivativeSymbolic, stateSymbolic);
B = jacobian(stateDerivativeSymbolic, inputSymbolic);

% Apply the hover condition
hoverEquilibrium = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
hoverInputs = [m * g; 0; 0; 0];

% Linearized A and B evaluated at hover
AHover = simplify(subs(A, [stateSymbolic; inputSymbolic], [hoverEquilibrium; hoverInputs]));
BHover = simplify(subs(B, [stateSymbolic; inputSymbolic], [hoverEquilibrium; hoverInputs]));

% Parameter set
mValue = 0.5;
gValue = 9.81;
IxValue = 0.02;
IyValue = 0.02;
IzValue = 0.04;


% Evaluate the state space matrices 
AHoverEvaluated = double(subs(AHover, {m, g}, {mValue, gValue}));
BHoverEvaluated = double(subs(BHover, {m, Ix, Iy, Iz}, {mValue, IxValue, IyValue, IzValue}));

% Assume full state feedback for this scenario
measurementMatrix = eye(12);

% Validate controllability
controllabilityTestMatrix = ctrb(AHoverEvaluated, BHoverEvaluated);
if rank(controllabilityTestMatrix) == length(AHoverEvaluated)
    disp('The quadcopter is controllable')
else
    disp('The quadcopter is not controllable')
end

% Validate observability
observabilityTestMatrix = obsv(AHoverEvaluated, measurementMatrix);
if rank(observabilityTestMatrix) == length(AHoverEvaluated)
     disp('The quadcopter is observable')
else
    disp('The quadcopter is not observable')
end  

% Save results to .mat for easy loading in other scripts
save('QuadcopterModel.mat','AHoverEvaluated','BHoverEvaluated', 'measurementMatrix', ...
     'mValue','gValue','IxValue','IyValue','IzValue');