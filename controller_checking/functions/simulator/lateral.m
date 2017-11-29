function [v, r] = lateral(u, v0, r0, steer, frequency)
    %% v0 and v in m/s, r0 and r and in rads
    %% u and steer are like the input arrays to the system
    a = 0.17;
    b = 0.16;
    Caf = 14.66;
    Car = Caf/b*a;
    Iz = 0.0333;
    
    A = [-(Caf+Car)/u, ((b*Car-a*Caf)/u-u);
         (b*Car-a*Caf)/(Iz*u), -(a^2*Caf + b^2*Car)/(Iz*u)];
    B = [Caf; a*Caf/Iz];

    state = [v0, r0];

    A_s = @(s) expm(s*A);
    Ad = A_s(1/frequency);
    Bd = integral(A_s, 0, 1/frequency, 'ArrayValued', true) * B;
    state = Ad*(state') + Bd*steer;

    v = state(1);
    r = state(2);
end

