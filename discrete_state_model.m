function [Ad, Bd, Cd, Dd] = discrete_state_model()
    % Get the constants from the general pool of constants
    constants = initial_constants();
    m     = constants{2}; 
    Iz    = constants{5}; 
    Caf   = constants{6};
    Car   = constants{7};
    lf    = constants{8};
    lr    = constants{9};
    Ts    = constants{10}; 
    Vx = constants{16}; % Vx
    
%     A = [-(2 * Caf + 2 * Car) / (m * Vx) 0 (-Vx - (2 * Caf * lf - 2 * Car * lr) / (m * Vx)) 0;
%          0 0 1 0;
%          -(2 * lf * Caf - 2 * lr * Car) / (Iz * Vx) 0 -(2 * lf^2 * Caf + 2 * lr^2 * Car) / (Iz * Vx) 0;
%          1 Vx 0 0];
    A = [0 1 Vx 0;
         0 -(2 * Caf + 2 * Car) / (m * Vx) 0 (-Vx - (2 * Caf * lf - 2 * Car * lr) / (m * Vx));
         0 0 0 1;
         0 -(2 * lf * Caf - 2 * lr * Car) / (Iz * Vx) 0 -(2 * lf^2 * Caf + 2 * lr^2 * Car) / (Iz * Vx)];
    B = [0; 2 * Caf / m; 0; 2 * lf * Caf / Iz];
    C = [1 0 0 0; 0 0 1 0];
    D = 0;
    
   %% Discretize the system
    
%     % Forward Euler
%     Ad=eye(length(A(1,:)))+Ts*A;
%     Bd=Ts*B;
%     Cd=C;
%     Dd=D;
    
    %Create state-space
    sysc = ss(A,B,C,D);
    sysd = c2d(sysc,Ts,'zoh');
    Ad = real(sysd.A);
    Bd = real(sysd.B);
    Cd = real(sysd.C);
    Dd = real(sysd.D);
    
end