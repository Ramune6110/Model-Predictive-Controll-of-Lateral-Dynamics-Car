function constants = initial_constants()
    % Constants    
    % Vehicle Dynamics and Control (2nd edition) by Rajesh Rajamani. They are in Chapter 2.3. 
    g = 9.81;
    m = 1500;
    
    rho = 1.225;
    Af  = 1.6 + 0.00056 * (m - 765);
    Cd  = 0.3;
    
    cd  = 0.5 * rho * Cd * Af;
    cr  = 0.03;
    Iz  = 3000;
    Caf = 19000;
    Car = 33000;
    lf  = 1.2;
    lr  = 1.6;
    Ts  = 0.1;
    
    Q = [10 0; 0 1]; % weights for outputs (output x output)
    S = [10 0; 0 1]; % weights for the final horizon outputs (output x output)
    R = 30;          % weights for inputs (input x input)
    
    % Reference Signalsの個数. 今回はpsiとYの二つなので2とする
    controlled_states = 2;
    
    hz = 15; % horizon period
    Vx = 20; % 横方向の制御のみなので速度は一定
    
    constants={g m cd cr Iz Caf Car lf lr Ts Q S R controlled_states hz Vx};
    
end