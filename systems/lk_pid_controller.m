 classdef lk_pid_controller < matlab.System & ...
                         matlab.system.mixin.Propagates & ...
                         matlab.system.mixin.Nondirect
  %Description:
  

  properties
    H_x = diag([1 0 0.5 0]);
    f_x = zeros(4,1);
    H_u = 4;
    f_u = 0;
  end

  properties(Nontunable)
    y_des = 0;       % desired lateral position
    K_p = 0.1;       % proportional weight
    K_d = 20;  % derivative weight
    K_i = 20;   % integral weight

    windup_reg = 0.1;  % integrate only absolute errors smaller than this number
    mem_length = 20; % number of timesteps to remember for computing derivative

    max_steering = 0.7854;  % max steering angle value
  end
  
  properties(DiscreteState)
    e_int;    % integrated error
    e_mem;    % saved error states
    e_mem_dt; % saved error times
    delta_f;
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
      obj.e_int = 0;
      obj.e_mem = zeros(1,20);
      obj.e_mem_dt = zeros(1,19);
      obj.delta_f = 0;

    end
    
    function ds = getDiscreteStateImpl(obj)
        % Return structure of properties with DiscreteState attribute
        ds.e_int = obj.e_int;
        ds.e_mem = obj.e_mem;
        ds.e_mem_dt = obj.e_mem_dt;
        ds.delta_f = obj.delta_f;
    end
    
    function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj, name)
        % Return size, data type, and complexity of discrete-state
        % specified in name
         if strcmp(name, 'e_int')
          sz = [1 1];
          dt = 'double';
          cp = false;
        elseif strcmp(name, 'e_mem')
          sz = [1 obj.mem_length];
          dt = 'double';
          cp = false;
        elseif strcmp(name, 'e_mem_dt')
          sz = [1 obj.mem_length-1];
          dt = 'double';
          cp = false;
        else
          sz = [1 1];
          dt = 'double';
          cp = false;
        end
    end

    % inputs
    function out = getNumInputsImpl(obj)
      out = 2;
    end
    function [o1,o2] = getInputNamesImpl(obj)
      o1 = 'lk_acc_state';
      o2 = 'dt';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 1;
    end
    function [n1] = getOutputNamesImpl(obj)
      n1 = 'deltaf';
    end
    function [o1] = getOutputDataTypeImpl(obj)
      o1 = 'double';
    end
    function [o1] = getOutputSizeImpl(obj)
      o1 = 1;
    end 
    function [f1] = isOutputFixedSizeImpl(obj)
       f1 = true;
    end    
    function [c1] = isOutputComplexImpl(obj)
       c1 = false;
    end
    % update
    function updateImpl(obj, state,dt)
      
      % error
      e = obj.y_des - state.y;
      
      % update memory buffer
      obj.e_mem = [obj.e_mem(2:end) e];
      obj.e_mem_dt = [obj.e_mem_dt(2:end) dt];
      % derivative error
      d_e = (obj.e_mem(end) - obj.e_mem(1))/sum(obj.e_mem_dt);
      % integral error  
      if abs(e) < obj.windup_reg
        % prevent excessive wind-up
        obj.e_int = obj.e_int + dt * e;
      end
      % control input
%       delta_f = 1;
%       disp(e);
%       disp(obj.K_p);
      obj.delta_f = 1 * e + obj.K_i * obj.e_int + obj.K_d * d_e;
%       disp(deltaf);
      % clip
%       delta_f = min(max(-obj.max_steering, delta_f), obj.max_steering);
     
    end

    function [delta_f] = outputImpl(obj, ~, ~)
        delta_f = obj.delta_f;
    end
    
    function [f1, f2] = isInputDirectFeedthroughImpl(~,~,~)
        f1 = false;
        f2 = false;
    end
  end
end