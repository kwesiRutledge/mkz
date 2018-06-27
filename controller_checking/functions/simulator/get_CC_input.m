classdef get_CC_input < matlab.System
    % Untitled3 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
    end

    properties(DiscreteState)
    end

    % Pre-computed constants
    properties(Access = private)        
        horizon = 20;
        Wv = 1/(0.01^2);
        Wi = 1/(0.05^2);
        Vr = 0.8;
        k = 0.2890;
        b = 0.0058;
        f0 = 0.6558;
        T = 1/10;
        Vmax = 2;
        Vmin = -0.2;
        Imax = 4.5;
        Imin = -3;
        V0 = 0;
        I0 = 4.5;
        
        H = 0;
        f = 0;
        A = 0;
        B = 0;
        Aeq = 0;
        Beq = 0;
        lb = 0;
        ub = 0;
        
        x0 = [];
        
        sync = false;
    end

    methods(Access = protected)
        function setupImpl(obj)
            obj.sync = false;
            % Perform one-time calculations, such as computing constants
                        % Perform one-time calculations, such as computing constants
            v_co = exp(-obj.b*obj.T);
            I_co = obj.k*(1-exp(-obj.b*obj.T))/obj.b;
            con_term = -obj.f0/obj.b*(1-exp(-obj.b*obj.T));
            
            %% H and f
            iWMat = 2*eye(obj.horizon+1)...
                    + [zeros(obj.horizon,1), -eye(obj.horizon);zeros(1,obj.horizon+1)]...
                    + [zeros(obj.horizon,1), -eye(obj.horizon);zeros(1,obj.horizon+1)]';
            iWMat(1) = 1;
            iWMat(end) = 1;
            iWMat = iWMat * obj.Wi;
            obj.H = 2 * blkdiag(obj.Wv*eye(obj.horizon+1), iWMat);
            obj.H(1,1) = 0;
            obj.f = [-2*obj.Vr*obj.Wv*ones(obj.horizon+1, 1); zeros(obj.horizon+1, 1)];
            

            %% Aeq and Beq
            v_constraints = [v_co*[eye(obj.horizon),zeros(obj.horizon,1)] - [zeros(obj.horizon,1), eye(obj.horizon)], I_co*[eye(obj.horizon),zeros(obj.horizon,1)]];
            obj.Aeq = [1, zeros(1, 2*obj.horizon+1);
                   v_constraints;
                   zeros(1, obj.horizon+1),1,zeros(1, obj.horizon);
                   zeros(obj.horizon, 2*(obj.horizon+1));];
            obj.Beq = [obj.V0; -con_term*ones(obj.horizon,1); obj.I0; zeros(obj.horizon,1)];

            %% A and B
            obj.A = blkdiag([-eye(obj.horizon+1);
                 eye(obj.horizon+1)],...
                 [-eye(obj.horizon+1);
                  eye(obj.horizon+1);]);
            obj.B = [-obj.Vmin*ones(obj.horizon + 1,1); obj.Vmax*ones(obj.horizon + 1,1);
                -obj.Imin*ones(obj.horizon + 1,1); obj.Imax*ones(obj.horizon + 1,1)];

            obj.lb = [obj.Vmin*ones(obj.horizon+1,1);obj.Imin*ones(obj.horizon+1,1)];
            obj.ub = [obj.Vmax*ones(obj.horizon+1,1);obj.Imax*ones(obj.horizon+1,1)];
        end

        function i = stepImpl(obj,v, vr)
            i = 0;
            obj.Vr = vr;
            if obj.sync == false
                i = obj.I0;
                obj.sync = true;
            else
                obj.sync = true;
                obj.Beq(1) = v; 
                obj.Beq(obj.horizon + 2) = obj.I0;
                obj.f = [-2*obj.Vr*obj.Wv*ones(obj.horizon+1, 1); zeros(obj.horizon+1, 1)];
%                 options = optimoptions(@quadprog,'Display', 'off');
%                 options = optimoptions(@quadprog,'OptimalityTolerance',1e-4, 'StepTolerance', 1e-3, 'Display', 'off');
                obj.x0 = quadprog(obj.H,obj.f,obj.A,obj.B,obj.Aeq,obj.Beq);
                if ~any(obj.x0)
                    disp('No solution')
                    i = 0;
                else
                    obj.I0 = obj.x0(obj.horizon + 1 + 2);
                    i = obj.x0(obj.horizon + 1 + 2);
                end
            end
            disp(['CC I:', num2str(i)])
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
