function [Aeq, Beq] = dyn2AeqBeq(A, B, E, K, x0, N, dt)
% dx/dt = A*x + B*u + E*d + K
% Here for simplicity, the input argument E = E*d
% x0 = [y0, v0, delta_yaw0, r0, steer0] for lane keeping

    if (size(A,2)+size(B,2))~= length(x0)
        error('Dimension of initial condition and A, B matrices mismatch')
    end
    A_s = @(s) expm(s*A);
    Ad = A_s(dt);
    integral_A = integral(A_s, 0, dt, 'ArrayValued', true);
    Bd = integral_A * B;
    EdKd = integral_A * (E+K); %Ed + Kd

    n = size(Ad,2);
    m = size(Bd,2);
    Ad = blkdiag(Ad, zeros(m));
    repB = zeros(n+m,n+m);
    repB(1:n, end-m+1:end) = Bd;

    unitMatA = [zeros(1, N+1); eye(N), zeros(N, 1)];
    unitMatB = blkdiag(0, eye(N));
    Aeq = kron(-Ad, unitMatA) + kron(-repB, unitMatB) + kron(blkdiag(eye(n),zeros(m)), eye(N+1)) + kron(blkdiag(zeros(n,n),1), blkdiag(1,zeros(N,N)));
    Beq = kron([EdKd;0], [0; ones(N,1)]) + kron(x0', [1; zeros(N,1)]);
end