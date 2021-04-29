%% Main file for controllering the Lateral car
clear
close all
clc

%% Load constaints
constants = initial_constants();
Ts = constants{10};
controlled_states = constants{14}; % number of controlled states in this script
hz = constants{15}; % horizon period
x_dot = constants{16};

%% Load the trajectory
t = 0:Ts:7; % Use 7 second for lane change
r = constants{17};
f = constants{18};
[psi_ref, X_ref, Y_ref] = trajectory_generator(t, r, f);

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