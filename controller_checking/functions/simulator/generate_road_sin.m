function road = generate_road_sin(t, rd_max, u_max)
    tspan = t(1):0.001:t(end);
    f = 1/5;
    rd = rd_max*sin(2*pi*f*tspan);
    u = u_max*ones(1,length(tspan));
    [t,X] = ode45(@(t, X) trajectory(t, X, u,rd, tspan), tspan, [0 0 0]);
    road = X(:,1:2);
end