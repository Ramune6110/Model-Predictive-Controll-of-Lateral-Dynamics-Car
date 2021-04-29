function [U1, old_states] = PID_Controll(psi_ref, Y_ref, states, old_states, i)
    constants = initial_constants();
    
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
    end
    
    old_states = states;
end