%% polysync_controller: control MKZ over polysync bus
function polysync_controller()
  RTK_SENSOR_ID = 1;    % sensor id of RTK GPS
  STOP_DISTANCE = 20;   % start braking [m]
  ST_RATIO = 12;        % steering ratio of car

  WHEEL_RADIUS = 0.24;  % wheel radius [m]

  BRAKE_MAX = 0.3;      % maximal braking when stopping
  BRAKE_TIME = 5;       % brake ramp time [s]

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

  sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage', ...
                               'SensorId', RTK_SENSOR_ID);

  sub_wsr = polysync.Subscriber('MessageType', 'PlatformWheelSpeedReportMessage');

  % Set up systems
  rd = road;
  rd.pathfile = '../mcity/highway.ascii';
  rd.circular = 0;
  rd.setup(struct());

  ACC = acc_pid_controller;
  ACC.mu_des = 26/3.6;
  ACC.max_throttle = 0.28;
  ACC.setup(struct(), 0.0);

  LK = lk_pcis_controller;
  LK.H_u = 100;   % weight in QP for steering (larger -> less aggressive centering)
  LK.setup(struct());

  % phase 0: shifting to D
  % phase 1: following route
  % phase 2: stopping
  % phase 3: shifting to P

  % timing
  last_timestamp = 0;

  % Phase variables
  brake_com = 0;    % braking phase
  phase = uint8(0);

  % Save last time
  last_time = embedded.fi(0, 'Signedness', 'Unsigned', ...
                          'FractionLength', 0, ...
                          'WordLength', 64);

  % Current wheel speed
  wheel_sp = single(0);

  % Shift to D
  shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, 0.01);

  phase = phase + 1;

  % Control loop
  while phase < uint8(3)
    % Read data
    [idx1, msg_mo] = sub_mo.step();
    [idx2, msg_wsr] = sub_wsr.step();

    if idx2 > 0
      wheel_sp = WHEEL_RADIUS * ...
             (msg_wsr.FrontLeft + msg_wsr.FrontRight + ...
              msg_wsr.RearLeft + msg_wsr.RearRight)/4;
    end

    if idx1 > 0

      % Take care of timing
      dt = 0;
      if last_time ~= embedded.fi(0, 'Signedness', 'Unsigned', ...
                                  'FractionLength', 0, ...
                                  'WordLength', 64);
        dt = double(msg_mo.Timestamp - last_time)/1e6;
      end
      last_time = msg_mo.Timestamp;

      % Convert data
      rawdata = get_data(msg_mo);

      % Tranform data to model states
      [lk_acc_state, road_left] = rd.step(rawdata);

      if road_left < STOP_DISTANCE
        phase = phase + 1;
      end

      % Compute model inputs
      [delta_f, lk_info] = LK.step(lk_acc_state);

      lk_acc_state
      delta_f
      lk_info

      if phase == uint8(1)
        % ACC controls speed
        [throttle_com] = ACC.step(lk_acc_state, dt);
        pub_msg = get_ba_message([], throttle_com, ST_RATIO*delta_f);

      elseif phase == uint8(2)
        % Braking phase
        brake_com = max(brake_com + BRAKE_MAX*dt/BRAKE_TIME, BRAKE_MAX);
        pub_msg = get_ba_message(brake_com, [], ST_RATIO*delta_f);

        if abs(wheel_sp) <  1e-2
          phase = phase + 1;
        end

      else
        pub_msg = get_ba_message();
      end
          
      pub_msg.Header.Timestamp = polysync.GetTimestamp;
      pub.step(pub_msg);    

      polysync.Sleep(0.01);
    end
  end

  shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, 0.01);
end
