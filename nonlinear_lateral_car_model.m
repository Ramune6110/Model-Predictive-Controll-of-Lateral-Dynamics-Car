function dx = nonlinear_lateral_car_model(t, states, U)
    % Get the constants from the general pool of constants
    constants = initial_constants();
    m     = constants{2}; 
    Iz    = constants{5}; 
    Caf   = constants{6};
    Car   = constants{7};
    lf    = constants{8};
    lr    = constants{9};
    x_dot = constants{16};
    
    % Assign the states
    y_dot   = states(1);
    psi     = states(2);
    psi_dot = states(3);
    Y       = states(4);
    
    % Input (steering wheel):
    U1  = U(1);
    
    % The nonlinear equation describing the dynamics of the drone
    dx(1,1) = -(2*Caf+2*Car)/(m*x_dot)*y_dot+(-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot))*psi_dot+2*Caf/m*U1;
    dx(2,1) = psi_dot;
    dx(3,1) = -(2*lf*Caf-2*lr*Car)/(Iz*x_dot)*y_dot-(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot)*psi_dot+2*lf*Caf/Iz*U1;
    dx(4,1) = sin(psi)*x_dot+cos(psi)*y_dot;
end