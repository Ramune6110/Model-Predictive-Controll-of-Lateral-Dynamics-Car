function [] = DrowResults(X_ref, Y_ref, psi_ref, states_total, UTotal, sim_length, t)
    %% Plot the trajectory
    % Trajectory
    figure(1);
    plot(X_ref(:,2),Y_ref(:,2),'--b','LineWidth',2)
    hold on
    plot(X_ref(:,2),states_total(1:end,1),'r','LineWidth',1)
    grid on;
    xlabel('x-position [m]','FontSize',15)
    ylabel('y-position [m]','FontSize',15)
    legend({'position-ref','position'},'Location','northeast','FontSize',15)

    %% Plot the position states and the yaw
    % Plot references individually
    figure;
    subplot(2,1,1)
    plot(t(1:sim_length),Y_ref(1:sim_length,2),'--b','LineWidth',2)
    hold on
    subplot(2,1,2)
    plot(t(1:sim_length),psi_ref(1:sim_length,2),'--b','LineWidth',2)

    hold on
    subplot(2,1,1)
    plot(t(1:sim_length),states_total(1:sim_length,1),'r','LineWidth',1)
    grid on
    xlabel('time [s]','FontSize',15)
    ylabel('y-position [m]','FontSize',15)
    legend({'y-ref','y-position'},'Location','northeast','FontSize',15)
    subplot(2,1,2)
    plot(t(1:sim_length),states_total(1:sim_length,3),'r','LineWidth',1)
    grid on
    legend({'psi-ref','psi-position'},'Location','northeast','FontSize',15)
    xlabel('time [s]','FontSize',15)
    ylabel('psi-position [rad]','FontSize',15)

    %% Plot the inputs
    figure;
    plot(t(1:length(states_total(:,1))),UTotal(:,1))
    grid on
    xlabel('time [s]','FontSize',15)
    ylabel('U2 [rad]','FontSize',15)
end