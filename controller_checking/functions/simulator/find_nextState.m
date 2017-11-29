function [X, Y, psi, u, v, r] = find_nextState(X, Y, psi, u, v, r, I, steer, dT)
    % r and psi have different sign
    frequency = 1/(dT/10);
    for i = dT/10:dT/10:dT
        u = longitudinal(u, I, frequency);
        [v, r] = lateral(u, v, r, steer, frequency);
        
        tspan = [0, 1/frequency];
        [t,x] = ode45(@(t, x) trajectory(t, x, [u,u],[-r,-r], tspan), tspan, [X Y psi]);
        X = x(end,1);
        Y = x(end,2);
        psi = x(end,3);
    end
end