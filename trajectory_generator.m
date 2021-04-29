function [psi_ref, X_ref, Y_ref] = trajectory_generator(t)
    constants = initial_constants();
    Ts = constants{10}; 
    x_dot = constants{16};
    
    % linspace 
    % ex) 区間 [-5,5] 内の 7 個の等間隔の点のベクトルを作成します
    % y1 = linspace(-5,5,7)
    
    x = linspace(0, x_dot * t(end), length(t));
    y = 3 * tanh((t - t(end) / 2));
    
    dx = x(2:end) - x(1:end-1);
    dy = y(2:end) - y(1:end-1);

    psi    = zeros(1,length(x));
    psiInt = psi;

    psi(1)     = atan2(dy(1),dx(1));
    psi(2:end) = atan2(dy(:),dx(:));
    dpsi       = psi(2:end) - psi(1:end-1);

    psiInt(1) = psi(1);
    for i = 2:length(psiInt)
        if dpsi(i - 1) < -pi
            psiInt(i) = psiInt(i - 1) + (dpsi(i - 1) + 2 * pi);
        elseif dpsi(i - 1) > pi
            psiInt(i) = psiInt(i - 1) + (dpsi(i - 1) - 2 * pi);
        else
            psiInt(i) = psiInt(i - 1) + dpsi(i - 1);
        end
    end

    X_ref   = [t' x'];
    Y_ref   = [t' y'];
    psi_ref = [t' psiInt'];
end