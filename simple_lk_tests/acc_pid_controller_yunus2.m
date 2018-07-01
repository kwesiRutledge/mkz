 classdef acc_pid_controller_yunus2 < matlab.System & ...
                         matlab.system.mixin.Propagates
  properties(Nontunable)
    mu_des = 10/3.6;       % desired speed
    K_p = 0.1*0.25;       % proportional weight
    K_d = 3*0.25*0.7/40;  % derivative weight
    K_i = 1.2*0.25/0.7;   % integral weight

    windup_reg = 2;  % integrate only absolute errors smaller than this number
    mem_length = 20; % number of timesteps to remember for computing derivative
    throttle_step = 0.27;
    stab_thres = 0.5;
    max_throttle = 0.28;  % max output value 
  end
  
  properties(DiscreteState)
    e_int;    % integrated error
    e_mem;    % saved error states
    e_mem_dt; % saved error times
    stab_flag;  % satabilization flag
    my_step;
  end

  methods
    function obj = acc_pid_controller_yunus2(varargin)
    end
  end

  methods(Access = protected)
    function setupImpl(obj)
      obj.e_int = 0;
      obj.e_mem = zeros(1,obj.mem_length);
      obj.e_mem_dt = zeros(1,obj.mem_length-1);
      obj.stab_flag = 0;
      obj.my_step = 0;
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
        elseif strcmp(name, 'stab_flag')
          sz = [1 1];
          dt = 'double';
          cp = false;
        elseif strcmp(name, 'my_step')
          sz = [1 1];
          dt = 'double';
          cp = false;
        end
    end

    % inputs
    function out = getNumInputsImpl(obj)
      out = 2;
    end
    function [o1, o2] = getInputNamesImpl(obj)
      o1 = 'lk_acc_state';
      o2 = 'dt';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 1;
    end
    function [n1] = getOutputNamesImpl(obj)
      n1 = 'throttle';
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
    function throttle = stepImpl(obj, state, dt)
%       disp(obj.mu_des);
      obj.my_step = obj.my_step +1;
      e = obj.mu_des - state.mu;
      obj.e_mem = [obj.e_mem(2:end) e];
      obj.e_mem_dt = [obj.e_mem_dt(2:end) dt];

      d_e = (obj.e_mem(end) - obj.e_mem(1))/sum(obj.e_mem_dt);

      if abs(e) < obj.windup_reg
        % prevent excessive wind-up
        obj.e_int = obj.e_int + dt * e;
      end

      throttle = obj.K_p * e + obj.K_i * obj.e_int + obj.K_d * d_e;
      
      
      if sum(abs(obj.e_mem))/obj.mem_length < obj.stab_thres && obj.my_step > obj.mem_length
        obj.stab_flag = 1;
        last_mu = state.mu
        last_throttle = throttle
      end
      if obj.stab_flag > 0
          throttle = obj.throttle_step;
      %    1;
      end
      throttle = max(0, throttle);
      throttle = min(obj.max_throttle, throttle);
    end

    function [f1, f2] = isInputDirectFeedthroughImpl(obj, ~, ~)
        f1 = true;
        f2 = true;
    end
  end
end
