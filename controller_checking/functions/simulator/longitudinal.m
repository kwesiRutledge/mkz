function u = longitudinal(u0, I, frequency)
    %Description:
    %   Updates the longitudinal velocity based on a simplified model
    %   of the car's longitudinal dynamics. There is a damping term
    %   along with a spring like input term and a constant in the update
    %   equations.
    %       dot-u[t] = -b * u[t] + k * I[t] - f0

    k = 0.2890;
    b = 0.0058;
    f0 = 0.6558;
    
    A = -b;
    B = k;
    K = -f0;
    
    A_s = @(s) exp(s*A);
    Ad = A_s(1/frequency);
    inte = integral(A_s, 0, 1/frequency);
    Bd = inte*B;
    Kd = inte*K;
    u = Ad*u0 + Bd*I + Kd;    
end