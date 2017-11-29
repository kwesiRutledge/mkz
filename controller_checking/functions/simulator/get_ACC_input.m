classdef get_ACC_input < matlab.System
    % Untitled3 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        N = 20;
        Wv = 1/(0.01^2);
        Wi = 1/(0.1^2);
        Wh = 1/(0.012^2);
    end

    properties(DiscreteState)
    end

    % Pre-computed constants
    properties(Access = private)
        M = 1000;
        k = 0.2890;
        b = 0.0058*10;
        f0 = 0.6558;
%         b = 0.0953;
%         f0 = 0.5246;
        con_term = 0;
        T = 1/10;
        Tgap = 0.5;
        hstop = 0.3; % When headway <= hstop, then stop.
        Vmax = 0.8;
        Vmin = -0.02;
        Imax = 4;
        Imin = -3;
        hmin = 0.3;
        
        VL = 0;
        V0 = 0;
        I0 = 2.2;
        h = 0;
        
        H = 0;
        f = 0;
        A = 0;
        B = 0;
        Aeq = 0;
        Beq = 0;
        
        x0 = [];
        sync = false;
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            %% Aeq and Beq

%             v_co = exp(-obj.b*obj.T);
%             I_co = obj.k*(1-exp(-obj.b*obj.T))/obj.b;
%             obj.con_term = -obj.f0/obj.b*(1-exp(-obj.b*obj.T));
%             
%             v_constraints = [v_co*[eye(obj.N),zeros(obj.N,1)] - [zeros(obj.N,1), eye(obj.N)], zeros(obj.N, obj.N+1), I_co*[eye(obj.N),zeros(obj.N,1)]];
%             h_constraints = [obj.Tgap/2*(-[eye(obj.N),zeros(obj.N,1)] - [zeros(obj.N,1), eye(obj.N)]), ([eye(obj.N),zeros(obj.N,1)] - [zeros(obj.N,1), eye(obj.N)]), zeros(obj.N, obj.N+1)];
%             obj.Aeq = [1, zeros(1, 3*obj.N+2);
%                        v_constraints;
%                        zeros(1, (obj.N+1)),1,zeros(1, 2*obj.N+1);
%                        h_constraints;
%                        zeros(1, 2*(obj.N+1)),1,zeros(1, obj.N);
%                        zeros(obj.N, 3*(obj.N+1))];
%             obj.Beq = [obj.V0; -obj.con_term*ones(obj.N,1); obj.h; -obj.VL*obj.Tgap*ones(obj.N,1); obj.I0; zeros(obj.N,1)];

            %% A and B
            obj.A = blkdiag([-eye(obj.N+1);
                    eye(obj.N+1)],...
                    -eye(obj.N+1),...
                     [-eye(obj.N+1);
                      eye(obj.N+1);]);
            obj.B = [obj.M; -obj.Vmin*ones(obj.N,1); obj.M; obj.Vmax*ones(obj.N,1);
                    obj.M; -obj.hmin*ones(obj.N,1);
                    obj.M; -obj.Imin*ones(obj.N,1); obj.M; obj.Imax*ones(obj.N,1)];
        end

        function i = stepImpl(obj,v,h,VL)
            i = 0;
            if obj.sync == false
                obj.sync = true;
            else
                obj.VL = VL;
                obj.h = h;
                obj.V0 = v;
                
                dt = obj.T;

                A = [-obj.b, 0;
                    -1, 0];
                B = [obj.k; 0];
                K = [obj.f0; 0];
                E = [0; 1];

                A_s = @(s) expm(s*A);
                Ad = A_s(dt);
                integral_A = integral(A_s, 0, dt, 'ArrayValued', true);
                Bd = integral_A * B;
                Ed = integral_A * E * obj.VL;
                Kd = integral_A * K;

                n = size(A,1);
                Ad = blkdiag(Ad, 0);
                repB = zeros(n+1,n+1);
                repB(1:n, end) = Bd;

                x0 = [obj.V0, obj.h, obj.I0];
                unitMatA = [zeros(1, obj.N+1); eye(obj.N), zeros(obj.N, 1)];
                unitMatB = blkdiag(0, eye(obj.N));
                obj.Aeq = kron(-Ad, unitMatA) + kron(-repB, unitMatB) + kron(blkdiag(eye(n),0), eye(obj.N+1)) + kron(blkdiag(zeros(n,n),1), blkdiag(1,zeros(obj.N,obj.N)));
                obj.Beq = kron([Ed+Kd;0], [0; ones(obj.N,1)]) + kron(x0', [1; zeros(obj.N,1)]);
                
%                 if abs(VL - v) < 0.1
%                     Wv = 0;
%                 else
%                     Wv = obj.Wv;
%                 end
                
                iWMat = 2*eye(obj.N+1)...
                    + [zeros(obj.N,1), -eye(obj.N);zeros(1,obj.N+1)]...
                    + [zeros(obj.N,1), -eye(obj.N);zeros(1,obj.N+1)]';
                iWMat(1) = 1;
                iWMat(end) = 1;
                iWMat = iWMat * obj.Wi;
                vhWMat = [(obj.Wv+obj.Wh*obj.Tgap^2)*eye(obj.N+1), -(obj.Wh*obj.Tgap)*eye(obj.N+1);
                          -(obj.Wh*obj.Tgap)*eye(obj.N+1), obj.Wh*eye(obj.N+1);];
                obj.H = 2 * blkdiag(vhWMat, iWMat);
                obj.f = [(-2*obj.VL*obj.Wv + 2*obj.hstop*obj.Tgap*obj.Wh)*ones(obj.N+1, 1);
                         -2*obj.hstop*obj.Wh*ones(obj.N+1, 1);
                         zeros(obj.N+1, 1)];
%                 obj.Beq = [v; -obj.con_term*ones(obj.N,1); obj.h; -obj.VL*obj.Tgap*ones(obj.N,1); obj.I0; zeros(obj.N,1)];

    %             options = optimoptions(@quadprog,'OptimalityTolerance',1e-5, 'StepTolerance', 1e-3, 'Display', 'off');
                obj.x0 = quadprog(obj.H,obj.f,obj.A,obj.B,obj.Aeq,obj.Beq);
                
                
                if ~any(obj.x0)
                    disp('No solution')
                    obj.I0 = 1;
                else
                    obj.I0 = obj.x0(2*obj.N + 4);
                end
            end
            i = obj.I0;
            disp(['ACC I:', num2str(i)])
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
