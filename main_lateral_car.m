%% Main file for controllering the Lateral car
clear
close all
clc

%% Load constaints
constants         = initial_constants();
Ts                = constants{10};
controlled_states = constants{14}; % number of controlled states in this script
hz                = constants{15}; % horizon period

%% Load the trajectory
t = 0:Ts:7; % Use 7 second for lane change
[psi_ref, X_ref, Y_ref] = trajectory_generator(t);

sim_length = length(t); % Number of control loop iterations

refSignals = zeros(length(X_ref(:, 2)) * controlled_states, 1);

k = 1;
for i = 1:controlled_states:length(refSignals)
    refSignals(i)     = psi_ref(k, 2);
    refSignals(i + 1) = Y_ref(k, 2);
    k = k + 1;
end

%% Load the initial state
y_dot   = 0;
psi     = psi_ref(1, 2);
psi_dot = 0;
Y       = Y_ref(1, 2);

% 状態量は以下の4つ
% Vehicle Dynamics and Control (2nd edition) by Rajesh Rajamani. They are in Chapter 2.3. 
states = [y_dot, psi, psi_dot, Y];
old_states = states;
% 全ての状態を保存する配列。各時間毎に保存。plotに使用
states_total = zeros(length(t), length(states));
states_total(1, :) = states;

%% Initiate the controller - simulation loops
% 今回横方向のみの制御のため制御指令値は舵角(δ)のみｓ
U1 = 0;
UTotal = zeros(length(t), 1);
UTotal(1, :) = U1;
            
k = 1; % for reading reference signals

tic;
for i = 1:sim_length - 1
    %% Model Predictive Controll
    %%%%%%%%%%%%%%%% Model Predictive Controll %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Generate discrete LPV Ad, Bd, Cd, Dd matrices
    [Ad, Bd, Cd, Dd] = discrete_state_model();
    
    %% Generating the current state and the reference vector
    x_aug_t = [states'; U1];
    
    k = k + controlled_states;
    if k + controlled_states * hz - 1 <= length(refSignals)
        r = refSignals(k:k + controlled_states * hz - 1);
    else
        r = refSignals(k:length(refSignals));
        hz = hz - 1;
    end
    
    %% Generate simplification matrices for the cost function
   [Hdb, Fdbt, Cdb, Adc] = MPC_simplification(Ad, Bd, Cd, Dd, hz);
   
   %% Calling the optimizer (quadprog)
    % Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du
    ft = [x_aug_t', r'] * Fdbt;
    
    % Hdb must be positive definite for the problem to have finite minimum
    % Call the solver
    options = optimoptions('quadprog','Display','off');
    lb  = -ones(1, hz) * pi/60;
    ub  = ones(1, hz) * pi/60;
    Hdb = real(Hdb);
    Hdb = (Hdb + Hdb') / 2;
    ft  = real(ft);
    [du, fval] = quadprog(Hdb, ft, [], [], [], [], lb, ub, [], options);
    
    % Update the real inputs
    U1 = U1 + du(1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% PID Controll
    %%%%%%%%%%%%%%%%%%%%%%%%% PID Controll %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PID_switch = constants{17};
    
    if PID_switch == 1
        if i == 1
            e_int_pid_yaw = 0;
            e_int_pid_Y   = 0;
        elseif i > 1
            e_pid_yaw_im1 = psi_ref(i - 1, 2) - old_states(1, 2);
            e_pid_yaw_i   = psi_ref(i, 2) - states(1, 2);
            e_dot_pid_yaw = (e_pid_yaw_i - e_pid_yaw_im1) / Ts;
            e_int_pid_yaw = e_int_pid_yaw + (e_pid_yaw_im1 + e_pid_yaw_i) / 2 * Ts;
            Kp_yaw = constants{18};
            Kd_yaw = constants{19};
            Ki_yaw = constants{20};
            U1_yaw = Kp_yaw * e_pid_yaw_i + Kd_yaw * e_dot_pid_yaw + Ki_yaw * e_int_pid_yaw;

            e_pid_Y_im1 = Y_ref(i - 1, 2) - old_states(1, 4);
            e_pid_Y_i   = Y_ref(i, 2) - states(1, 4);
            e_dot_pid_Y = (e_pid_Y_i - e_pid_Y_im1) / Ts;
            e_int_pid_Y = e_int_pid_Y + (e_pid_Y_im1 + e_pid_Y_i) / 2 * Ts;
            Kp_Y = constants{21};
            Kd_Y = constants{22};
            Ki_Y = constants{23};
            U1_Y = Kp_Y * e_pid_Y_i + Kd_Y * e_dot_pid_Y + Ki_Y * e_int_pid_Y;

            U1 = U1_yaw + U1_Y;
            
            if U1 < -pi/6
                U1 = -pi/6;
            elseif U1 > pi/6
                U1 = pi/6;
            end
            
            old_states = states;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % MPCもしくはPIDのどちらかで計算した制御量を保存
    UTotal(i + 1, :) = U1;
    
    % Simulate the new states
    T = Ts * i: Ts / 30 : Ts * i + Ts;
    [T,x] = ode45(@(t,x) nonlinear_lateral_car_model(t, x, U1), T, states);
    states = x(end,:);
    states_total(i + 1, :) = states;
    
    imaginary_check = imag(states) ~= 0;
    imaginary_check_sum = sum(imaginary_check);
    if imaginary_check_sum ~= 0
        disp('Imaginary part exists - something is wrong');
    end

end
toc

disp("simulation end!");

% Plot the results
DrowResults(X_ref, Y_ref, psi_ref, states_total, UTotal, sim_length, t);
