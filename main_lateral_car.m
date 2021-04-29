%% Main file for controllering the Lateral car
clear
close all
clc

%% Load constaints
constants         = initial_constants();
Ts                = constants{10};
controlled_states = constants{14}; % number of controlled states in this script
hz                = constants{15}; % horizon period
x_dot             = constants{16};

%% Load the trajectory
t = 0:Ts:7; % Use 7 second for lane change
r = constants{17};
f = constants{18};
[psi_ref, X_ref, Y_ref] = trajectory_generator(t, r, f);

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
% 全ての状態を保存する配列。各時間毎に保存。plotに使用
states_total = zeros(length(t), length(states));
states_total(1, :) = states;

%% Initiate the controller - simulation loops
% 今回横方向のみの制御のため制御指令値は舵角(δ)のみｓ
U1 = 0;
UTotal = zeros(length(t), 1);
UTotal(1, :) = U1;

k = 1; % for reading reference signals
for i = 1:sim_length - 1
    %% Generate discrete LPV Ad, Bd, Cd, Dd matrices
    [Ad, Bd, Cd, Dd] = discrete_state_model(states);
    
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
   
   
end
%% Plot the trajectory
% Trajectory
figure(1);
plot(X_ref(:,2),Y_ref(:,2),'--b','LineWidth',2)
hold on
% plot(X_ref(:,2),statesTotal(1:end,4),'r','LineWidth',1)
grid on;
xlabel('x-position [m]','FontSize',15)
ylabel('y-position [m]','FontSize',15)
legend({'position-ref','position'},'Location','northeast','FontSize',15)