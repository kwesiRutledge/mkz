classdef get_lat_input < matlab.System
    % Untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        %% Controller parameters
        N = 20;
        Wy = 10;
        Wv = 10;
        Wp = 10;
        Wr = 20;
        Ws = 10;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        M = 1000; % A large number
        %% Car parameters
        T = 1/10;
        a = 0.16;
        b = 0.17;
        m = 4.5148;
        Caf = 14.66;
        Car = 14.66/0.16*0.17;
        Iz = 0.0833/2.5;
        L = 0.36;
        X;
        
        %% Initial conditions & messages
        u_m = 0;
        r_m = 0;
        s0 = 0;
        range = [0.15; 0.5; 10/180*pi; 20/180*pi; 200*atan(12.5/147)/70];
%         range = [10; 10; 10; 10; 10];
        %% Control msgs
        servomsg;
        servopub;
        CPS2V = 0.004641;
    end

    methods(Access = protected)
        function setupImpl(obj)
            %% Set initial conditions
            obj.s0 = 0;
        end

        function steer = stepImpl(obj, u, y, v, delta_yaw, r, rd)
            %% Extract the feedback
            if u > 0.01
                steer0 = obj.s0;

                %% Initializing Matrices
                dt = obj.T;

                a = 0.17;
                b = 0.16;

                Caf = 14.66;
                Car = Caf*a/b;
                Iz = 0.0833/2.5;

                A = [0, 1, u, 0;
                     0, -(Caf + Car)/u, 0, ((-a*Caf + b*Car)/u - u);
                     0, 0, 0, 1;
                     0, (-a*Caf + b*Car)/(Iz*u), 0, -(a^2*Caf + b^2*Car)/(Iz*u)];
                B = [0; Caf; 0; a*Caf/Iz];
                E = [0; 0; -1; 0];
                K = [0; 0; 0; 0];

%                 A_s = @(s) expm(s*A);
%                 Ad = A_s(dt);
%                 integral_A = integral(A_s, 0, dt, 'ArrayValued', true);
%                 Bd = integral_A * B;
%                 Ed = integral_A * E * rd;
% 
%                 Ad = blkdiag(Ad, 0);
%                 repB = zeros(5,5);
%                 repB(1:4, 5) = Bd;

                x0 = [y, v, delta_yaw, r, steer0];
                [Aeq, Beq] = dyn2AeqBeq(A, B, E*rd, K, x0, obj.N, obj.T);
%                 unitMatA = [zeros(1, obj.N+1); eye(obj.N), zeros(obj.N, 1)];
%                 unitMatB = blkdiag(0, eye(obj.N));
%                 Aeq = kron(-Ad, unitMatA) + kron(-repB, unitMatB) + kron(blkdiag(eye(4),0), eye(obj.N+1)) + kron(blkdiag(zeros(4,4),1), blkdiag(1,zeros(obj.N,obj.N)));
%                 Beq = kron([Ed;0], [0; ones(obj.N,1)]) + kron(x0', [1; zeros(obj.N,1)]);

                Aueq = kron(eye(5), [-eye(obj.N+1); eye(obj.N+1)]);
                Bueq = kron(obj.range, repmat([0; ones(obj.N,1)], 2, 1)) + kron(obj.M*ones(5,1), repmat([1; zeros(obj.N,1)],2,1));

                %% weight matrix for input
                iWMat = 2*eye(obj.N+1)...
                        + [zeros(obj.N,1), -eye(obj.N);zeros(1,obj.N+1)]...
                        + [zeros(obj.N,1), -eye(obj.N);zeros(1,obj.N+1)]';
                iWMat(1) = 1;
                iWMat(end) = 1;

                H = 2*blkdiag(obj.Wy*eye(obj.N+1),...
                              obj.Wv*eye(obj.N+1),...
                              obj.Wp*eye(obj.N+1),...
                              obj.Wr*eye(obj.N+1),...
                              obj.Ws*iWMat);
                          
                f = kron([0; -2*obj.Wv*rd; 0; -2*obj.Wr*rd; 0], ones(obj.N+1, 1));
%                 H = 2*blkdiag(kron([obj.Wy, 0, -obj.Wy*obj.L;
%                                     0, obj.Wv, obj.Wv*u;
%                                     -obj.Wy*obj.L, obj.Wv*u, obj.Wv*u^2+obj.Wp+obj.Wy*obj.L^2], eye(obj.N+1)),...
%                               (obj.Wr)*eye(obj.N+1),...
%                               obj.Ws*iWMat);

%                 f = kron([0; 0; 0; -2*obj.Wr*rd; 0], ones(obj.N+1, 1));
                disp('LK_sys')
                [optx,fval,exitflag,output] = quadprog(H,f,Aueq,Bueq,Aeq,Beq);
                if ~any(optx)
                    disp('No solution')
%                     obj.s0 = 0;
                else
                    disp('Have a solution')
                    obj.s0 = optx(end-obj.N+1); %Extract the next steering command
                end
            end
            steer = obj.s0;
            disp(['LK_strict s:', num2str(steer*180/pi)])
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
