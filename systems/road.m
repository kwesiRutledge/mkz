classdef road < matlab.System & matlab.system.mixin.Propagates
  
  properties(Nontunable)
    lat0 = 42.30095833;     % local center latitude   [deg]
    long0 = -83.69758056;   % local center longitude  [deg]
    h0 = 0;                 % local center altitude
    pathfile = 'mcity/fixed_path.ascii';    % path in local (xE, yN)
    circular = 0;
    
    N_path = 3;             % number of points in each direction to fit
    N_interp = 10;          % number of subdivisions to find closes point
    dt = 0.1;               % time step for computing derivatives
  end

  properties(SetAccess = protected, GetAccess = public)
    len_path = 0;
    path_ = 0;
    size_path = 0;
  end
  
  methods
    function obj = road(varargin)
    end
  end
  
  methods(Access = protected)
    function setupImpl(obj)
      tt = coder.load(obj.pathfile);
      obj.path_ = tt;
      
      obj.size_path = size(obj.path_,1);
      obj.len_path = obj.path_(end,3);
    end
     % Error handling for input values
    function validateInputsImpl(~,in)
      if ~isstruct(in)
         error('Input must be struct');
      end
    end
    % inputs
    function num = getNumInputsImpl(obj)
      num = 1;
    end
    function out = getInputNamesImpl(obj)
      out = 'data';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 2;
    end
    function [o1, o2] = getOutputDataTypeImpl(obj)
      o1 = 'LKACCBus';
      o2 = 'double';
    end
    function [o1, o2] = getOutputSizeImpl(obj)
      o1 = 1;
      o2 = 1;
    end
    function [o1, o2] = getOutputNamesImpl(obj)
      o1 = 'lk_acc_state';
      o2 = 'road_left';
    end
    function [c1, c2] = isOutputComplexImpl(obj)
      c1 = false;
      c2 = false;
    end
    function [f1, f2] = isOutputFixedSizeImpl(obj)
      f1 = true;
      f2 = true;
    end
    
    function [lk_acc_state, road_left] = stepImpl(obj, data)
      % position of gps receiver in (xEast, yNorth)
      r_gps = get_vehicle_pos(obj, data.latitude, data.longitude, data.el);
      
      % vector gps receiver -> CG in (xEast, yNorth)
      r_gps_cg = [data.x_gps_cg * cos(data.Yaw) - data.y_gps_cg * sin(data.Yaw);
            data.x_gps_cg * sin(data.Yaw) + data.y_gps_cg * cos(data.Yaw)];    

      % position of CG in (xEast, yNorth)
      r_cg = r_gps + r_gps_cg;
      
      [rc, drc, kappa, road_left] = get_road_state(obj, r_cg);
      
      road_state.car_x = r_cg(1);
      road_state.car_y = r_cg(2);
      
      road_state.road_x = rc(1);
      road_state.road_y = rc(2);
      
      road_state.d_road_x = drc(1);
      road_state.d_road_y = drc(2);
      
      road_state.kappa = kappa;


      %%%% Transformations to ACC/LK state %%%%%
      % unit vector direction of road
      road_unit = [road_state.d_road_x; road_state.d_road_y]/...
            norm([road_state.d_road_x; road_state.d_road_y]);

      % relative position
      rel_pos = [road_state.car_x - road_state.road_x;
            road_state.car_y - road_state.road_y];
      
      % velocity in global frame
      global_vel = [data.Vx*cos(data.Yaw)-data.Vy*sin(data.Yaw);
             data.Vx*sin(data.Yaw)+data.Vy*cos(data.Yaw)];
      
      % yaw angles
      normDeg = @(x) pi-mod(pi-x, 2*pi);
      
      road_yaw = cart2pol(road_unit(1), road_unit(2));
      global_yaw = normDeg(normDeg(data.Yaw) + cart2pol(data.Vx, data.Vy));
      
      dPsi = normDeg(global_yaw - road_yaw);
      
      % assign
      lk_acc_state.y = det([road_unit rel_pos]);
      lk_acc_state.dy = det([road_unit global_vel]);
      lk_acc_state.mu = data.Vx;
      lk_acc_state.nu = data.Vy;
      lk_acc_state.dPsi = dPsi;
      lk_acc_state.r = data.YawRate;
      lk_acc_state.h = 8;
      lk_acc_state.r_d = road_state.kappa * norm(global_vel);
    end
  end
  methods
    function [veh_pos] = get_vehicle_pos(obj, lat, long, h)
      % get position relative to local center
      
      % WGS 84 data
      a = 6378137;
      b = 6356752.31424518;
      f = (a-b)/a;
      e_sq = f*(2-f);
      
      cosPhi = cosd(lat);
      sinPhi = sind(lat);
      
      cosPhi0 = cosd(obj.lat0);
      sinPhi0 = sind(obj.lat0);
      cosLambda0 = cosd(obj.long0);
      sinLambda0 = sind(obj.long0);
      
      N = a/sqrt(1-e_sq*sinPhi^2);
      
      % ECEF coordinates
      x = (h+N) * cosPhi * cosd(long);
      y = (h+N) * cosPhi * sind(long);
      z = (h + (1-e_sq) * N) * sinPhi;
      
      N2 = a/sqrt(1-e_sq*sinPhi0^2);
      
      xd = x - (obj.h0 + N2) * cosPhi0 * cosLambda0;
      yd = y - (obj.h0 + N2) * cosPhi0 * sinLambda0;
      zd = z - (obj.h0 + (1-e_sq) * N2) * sinPhi0;
      
      t = cosLambda0 * xd + sinLambda0 * yd;
      
      % ENU coordinates
      xEast  = -sinLambda0 * xd + cosLambda0 * yd;
      zUp    =  cosPhi0 * t + sinPhi0 * zd;
      yNorth = -sinPhi0 * t + cosPhi0 * zd;
      
      veh_pos = [xEast; yNorth];
    end
    
    function [rc, drc, kappa, road_left] = get_road_state(obj, veh_pos)
      % get road states (rc: road center, drc: (d/dt) rc, kappa:
      % curvature)

      % find closest point on path
      pt_idx_min = min_idx( obj.path_(:, 1:2) - repmat(veh_pos', size(obj.path_,1), 1) );
      interp_ival = 1 + mod((pt_idx_min-obj.N_path:pt_idx_min+obj.N_path) - 1, ...
                            obj.size_path-1);
      
      if obj.circular
        % Path is a loop
        ival_mid = obj.N_path+1;

        interp_ival = 1 + mod((pt_idx_min-obj.N_path:pt_idx_min+obj.N_path) - 1, ...
          obj.size_path-1);

        s_interp = obj.path_(interp_ival, 3);
        s_interp = s_interp - obj.len_path.*(s_interp > s_interp(end));
      else
        % Path is not loop
        ival_sta = max(1, pt_idx_min-obj.N_path);
        ival_end = min(obj.size_path, pt_idx_min+obj.N_path);
        ival_mid = pt_idx_min-ival_sta;   

        interp_ival = ival_sta:ival_end; % points to interpolate

        s_interp = obj.path_(interp_ival, 3);
      end

      x_spline = spline(s_interp, obj.path_(interp_ival, 1));
      y_spline = spline(s_interp, obj.path_(interp_ival, 2));
      
      % find closest point along interpolated curve
      interp_pts = linspace(s_interp(max(1,ival_mid-1)), s_interp(min(length(s_interp), ival_mid+1)), obj.N_interp);
      traj_pts = [ppval(x_spline, interp_pts)' ppval(y_spline, interp_pts)'];
      s_idx_min = min_idx(traj_pts - repmat(veh_pos', obj.N_interp, 1));
      
      s = fminsearch(@(t) norm(veh_pos - [ppval(x_spline,t); ppval(y_spline, t)]), ...
                     interp_pts(s_idx_min));

      road_left = obj.len_path - s;

      % position
      rc = [ppval(x_spline, s);
         ppval(y_spline, s)];
      
      % 1st derivative of position
      drc = ([ppval(x_spline, s+obj.dt);
        ppval(y_spline, s+obj.dt)] ...
        - rc)/obj.dt;
      
      % 2nd derivative of position
      ddrc = ([ppval(x_spline, s+obj.dt) + ppval(x_spline, s-obj.dt);
        ppval(y_spline, s+obj.dt) + ppval(y_spline, s-obj.dt)] ...
        - 2*rc)/obj.dt^2;
      
      % curvature
      kappa = det([drc ddrc])/norm(drc)^3;
    end
    
    function [rc, drc, kappa] = get_pos(obj, s)
      % return point at distance s along path            
      [~, idx] = sort(abs(obj.path_(:,3) - s));
      s0 = obj.path_(idx(1),3);
      x0 = obj.path_(idx(1),1:2);
      
      s1 = obj.path_(idx(2),3);
      x1 = obj.path_(idx(2),1:2);

      x = x0 * (s-s0)/(s1-s0) + x1 * (s1-s)/(s1-s0);

      [rc, drc, kappa] = obj.get_road_state(x');
    end
  end
end

function [idx] = min_idx(v)
  % return index of row in v with smallest 2-norm
  [~, idx] = min(sum(v.^2, 2), [], 1);
end