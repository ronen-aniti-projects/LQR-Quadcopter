function stateDerivative = evaluateDynamics(parameterValues, stateValues, inputValues)
    
    % The params
    m = parameterValues(1);
    g = parameterValues(2); 
    Ix = parameterValues(3); 
    Iy = parameterValues(4);
    Iz = parameterValues(5);
    
    % The states
    x = stateValues(1);
    y = stateValues(2);
    z = stateValues(3);
    phi = stateValues(4);
    theta = stateValues(5);
    psi = stateValues(6);  
    xd = stateValues(7);
    yd = stateValues(8);
    zd = stateValues(9);
    p = stateValues(10);
    q = stateValues(11);
    r = stateValues(12);
    
    % The inputs
    Fc = inputValues(1);
    Mx = inputValues(2);
    My = inputValues(3);
    Mz = inputValues(4);

    % Evaluate the state derivative based on the nonlinear EOMs
    stateDerivative = zeros(12,1);
    stateDerivative(1) = xd;
    stateDerivative(2) = yd;
    stateDerivative(3) = zd;
    stateDerivative(4) = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
    stateDerivative(5) =  q*cos(phi) - r*sin(phi);
    stateDerivative(6) = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta);
    stateDerivative(7) =  (Fc*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/m;
    stateDerivative(8) = -(Fc*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/m;
    stateDerivative(9) =  (Fc*cos(phi)*cos(theta))/m - g; 
    stateDerivative(10) = Mx/Ix + q*r*(Iy - Iz)/Ix;
    stateDerivative(11) = My/Iy - p*r*(Ix - Iz)/Iy;
    stateDerivative(12) = Mz/Iz + p*q*(Ix - Iy)/Iz;

end