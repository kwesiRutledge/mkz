function dX = trajectory(t, X, u, r, tspan)
    u_t = interp1(tspan, u, t, 'spline');
    r_t = interp1(tspan, r, t, 'spline');
    dxdt = u_t*cos(X(3));
    dydt = u_t*sin(X(3));
    dphidt = r_t;

    dX = [dxdt;
        dydt;
        dphidt;];
end