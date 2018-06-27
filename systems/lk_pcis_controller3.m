 classdef lk_pcis_controller3 < matlab.System & ...
                         matlab.system.mixin.Propagates & ...
                         matlab.system.mixin.Nondirect
  %Description:
  % This constroller should automatically incorporate the previous buff_len of inputs
  % into its calculation of the next input. (We want smoothness of the input trajectory)
  % There will be a quadratic weight on the expression "u[t] - mean([u[t-1],u[t-2],...,u[t-buff_len] ])"
  % This should primarily affect the mpcqpsolver's objective function.

  properties
    H_x = diag([1 0 0.5 0]);
    f_x = zeros(4,1);
    H_u = 4;
    f_u = 0;
  end

  properties(Nontunable)
    M = 1800;
    lf = 1.2;
    lr = 1.65;
    Caf = 140000;
    Car = 120000;
    Iz = 3270;

    buff_len = 5;
    
    sol_opts = struct('DataType', 'double', 'MaxIter', 200, ...
                      'FeasibilityTol', 1e-6, 'IntegrityChecks', true);
  end

  properties(Access = private)
    B_lk;
    E_lk;
    data;
    delta_f_prev;   %Buffer containing the buff_len number of previous steering inputs
  end
  
  properties(DiscreteState)
    delta_f;
    barrier_val;
    qp_status;
  end

  methods
    function obj = lk_pcis_controller3(varargin)
      % lk_pcis_controller2: lane keeping controller based on a PCIS barrier.
      % 
      % Inputs: 
      %  - lk_acc_state: state of lane dynamics [Bus: LKACCBus]
      % 
      % Outputs: 
      %  - delta_f: steering angle [rad]
      %  - control_info   [Bus: ControlInfoBus] 
      %
      % Car parameters:
      %  - M: car mass [kg]
      %  - lf: distance from center of gravity to front axle [m]
      %  - lr: distance from center of gravity to rear axle [m]
      %  - Caf: cornering stiffness front tires [N/rad]
      %  - Iz: yaw moment of inertia [kg m^2]

    end
  end

  methods(Access = protected)
    function setupImpl(obj)
      obj.B_lk = [0; obj.Caf/obj.M; 0; obj.lf*obj.Caf/obj.Iz];
      obj.E_lk = [0; 0; -1; 0];
      data_temp = coder.load('lk_pcis_controller.mat');
      obj.data = data_temp;
      
      obj.delta_f = 0;
      obj.qp_status = 0;
      
      obj.barrier_val = -0.1;

      %Set Up Buffer of Previous input values
      obj.delta_f_prev = zeros(obj.buff_len,1);
      obj.qp_status = 0;
    end
    
    function ds = getDiscreteStateImpl(obj)
        % Return structure of properties with DiscreteState attribute
        ds.delta_f = obj.delta_f;
        ds.qp_status = obj.qp_status;
    end
    
    function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,~)
        % Return size, data type, and complexity of discrete-state
        % specified in name
        sz = [1 1];
        dt = 'double';
        cp = false;
    end

    % inputs
    function out = getNumInputsImpl(obj)
      out = 1;
    end
    function [o1] = getInputNamesImpl(obj)
      o1 = 'lk_acc_state';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 2;
    end
    function [n1, n2] = getOutputNamesImpl(obj)
      n1 = 'delta_f';
      n2 = 'control_info';
    end
    function [o1, o2] = getOutputDataTypeImpl(obj)
      o1 = 'double';
      o2 = 'ControlInfoBus';
    end
    function [o1, o2] = getOutputSizeImpl(obj)
      o1 = 1;
      o2 = 1;
    end 
    function [f1, f2] = isOutputFixedSizeImpl(obj)
       f1 = true;
       f2 = true;
    end    
    function [c1, c2] = isOutputComplexImpl(obj)
       c1 = false;
       c2 = false;
    end
    % update
    function updateImpl(obj, lk_acc_state)
      
      % Retrieve State
      % ++++++++++++++

      x_lk = [lk_acc_state.y; lk_acc_state.nu; lk_acc_state.dPsi; lk_acc_state.r];

      mu = lk_acc_state.mu;
      r_d = lk_acc_state.r_d;
      
      if max(abs(x_lk)) > 10
        % Obviously bogus data

        return
      end

      % If the speed is too low, use a simple Controller
      % ++++++++++++++++++++++++++++++++++++++++++++++++

      if mu < 3
        % Use very simple P controller
        obj.delta_f = -0.1*(x_lk(1)+0.05*x_lk(3));
        obj.qp_status = -1;
        return
      end
       
      % Define the System Matrices (given the current speed)
      % ++++++++++++++++++++++++++++++++++++++++++++++++++++

      A_lk = [0, 1, mu, 0; 
          0, -(obj.Caf+obj.Car)/obj.M/mu, 0, ((obj.lr*obj.Car-obj.lf*obj.Caf)/obj.M/mu - mu); 
          0, 0, 0, 1;
          0, (obj.lr*obj.Car-obj.lf*obj.Caf)/obj.Iz/mu,  0, -(obj.lf^2 * obj.Caf + obj.lr^2 * obj.Car)/obj.Iz/mu];
      
      dt = obj.data.con.dt;

      A = eye(4) + A_lk * dt + A_lk^2 * dt^2/2 + A_lk^3 * dt^3/3/2;

      A_int = eye(4)*dt + A_lk * dt^2/2 * A_lk^2 * dt^3/3/2 + A_lk^3 * dt^4/4/3/2;

      B = A_int * obj.B_lk;
      E = A_int * obj.E_lk;

      % A = A_lk*obj.data.con.dt + eye(4);
      % B = obj.B_lk*obj.data.con.dt;
      % E = obj.E_lk*obj.data.con.dt;
      K = zeros(4,1);
      
      % Define System Matrices for trajectories
      % +++++++++++++++++++++++++++++++++++++++

      sys0.A  = A;
      sys0.B  = B;
      sys0.K  = K;
      sys0.x0 = x_lk;

      sys0.C = eye(size(A,1));  %C is not going to be used here,
                                %but in order for my function to work
                                %I need it.

      [G,H,C_big,x0_mat] = create_skaf_n_boyd_matrices(sys0,obj.t_horizon);

      %define E-bar
      % for t = 1: obj.t_horizon
      %   temp_E_bar{t} = E;
      % end
      % E_bar = blkdiag(temp_E_bar{:});
      E_bar = kron(eye(obj.t_horizon),E);

      % Define the Objective's Cost Matrices
      % ++++++++++++++++++++++++++++++++++++ 

      %Make the cost matrix for the trajectory have equal weight for deviations
      %at any point in the time horizon.

      % for i = 1:obj.t_horizon+1
      %   temp_R_x{i} = obj.d_factor^i * ( obj.H_x );
      % end
      % R_x = blkdiag(temp_R_x{:});

      %Create Diagonal
      temp_diag = zeros(obj.t_horizon+1,1);
      for i = 1:obj.t_horizon+1
        temp_diag(i) = obj.d_factor^(i-1);
      end
      R_x = kron(diag(temp_diag),obj.H_x);

      r_x = repmat(obj.f_x,obj.t_horizon,1);

      R_u = obj.H_u;
      r_u = obj.f_u;

      % Retrieve the Safe Set's Polyhedral Representation
      % +++++++++++++++++++++++++++++++++++++++++++++++++

      A_x = obj.data.poly_A;
      b_x = obj.data.poly_b;

      m = size(A_x,1);
      
      % Apply MPC Quadratic Program Solver
      % ++++++++++++++++++++++++++++++++++

      H_cost = zeros(1,1);
      f_cost = zeros(m,1);
      
      H(1,1) = B'*R_x*B + R_u;
      f(1,1) = r_u + B'*R_x*(A*x_lk + K) + B'*R_x*E*r_d + B'*r_x;
      
      % size(G)
      % size(x0_mat)
      % size((blkdiag(repmat(E,1,1,obj.t_horizon))) )

      H_cost = ones(v*obj.t_horizon,v)'*H'*R_x*H*ones(obj.t_horizon*v,v) + R_u;
      f_cost = r_u + ones(v*obj.t_horizon,v)'*H'*R_x*(G*E_bar*repmat(r_d,obj.t_horizon,1)+x0_mat );

      A_constr = zeros(2*m+2,1);
      b_constr = zeros(2*m+2,1);
      
      A_constr(1:m,:) = A_x*B;
      A_constr(m+1:2*m,:) = A_x*B;
      
      A_constr(2*m+1:2*m+2,:) = [1;-1];
    
      b_constr(1:m,:)     = b_x -A_x*A*x_lk - A_x*E*( obj.data.con.rd_max);
      b_constr(m+1:2*m,:) = b_x -A_x*A*x_lk - A_x*E*(-obj.data.con.rd_max);

      b_constr(2*m+1:2*m+2,:) = [obj.data.con.df_max;
                                 obj.data.con.df_max];
 
      % Objective 0.5 x' H x + f' x
      L = chol(H, 'lower');
      Linv = zeros(1,1);
      Linv(1,1) = L\eye(size(H,1));
      
      [u, status] = mpcqpsolver(Linv, f, -A_constr, -b_constr, ...
                      [], zeros(0,1), ...
                      false(size(A_constr,1),1), obj.sol_opts);

      % disp(['status = ' num2str(status) ])

      % disp(['f = ' num2str(f) ', H = ' num2str(H)])
      obj.qp_status = status;

      % normalized distance from Polyhedron edge
      scale_vec = [obj.data.con.y_max obj.data.con.nu_max ...
                   obj.data.con.psi_max obj.data.con.r_max];
      Da = A_x.*repmat(scale_vec, size(A_x,1), 1);
      d_list = (b_x - A_x*x_lk)./sum(A_x.*A_x, 2) ...
                .*sqrt(sum(Da.*Da, 2));
      obj.barrier_val = min(d_list);

      if status > 0
        % qp solved successfully
        obj.delta_f = u;
      else
        %infeasible: keep control constant
        % status
        % x_lk
        % mu
        % r_d
        
        all(A_x*x_lk <= b_x);
        obj.barrier_val = -0.05;
      end

      %Update Buffer of Previous Inputs
      obj.delta_f_prev = [ obj.delta_f ; obj.delta_f_prev(2:end) ];
     
    end

    function [delta_f, control_info] = outputImpl(obj, ~, ~)
        delta_f = obj.delta_f;
        control_info.barrier_val = obj.barrier_val;
        control_info.ddy = 0;
        control_info.qp_status = obj.qp_status;
    end
    
    function [f1, f2] = isInputDirectFeedthroughImpl(~, ~)
        f1 = false;
        f2 = false;
    end
  end
end
