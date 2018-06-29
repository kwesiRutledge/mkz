%% polysync_controller: control MKZ over polysync bus
%  display_test()
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

  sub_brake = polysync.Subscriber('MessageType', 'PlatformBrakeReportMessage');


  % Phase variables
  phase = 0;
  phase = phase + 0.1;
  disp('MKZ_test')
  % Control loop
  while phase < 6
    % Read data

    [idx1, msg_mo] = sub_mo.step();
    [idx2, msg_wsr] = sub_wsr.step();
    [idx3, msg_br] = sub_brake.step()
  
    m = msg_mo.Acceleration';
    % b = msg_br.PedalInput;
    % disp(num2str(b));

    polysync.Sleep(0.1);
    phase = phase + 0.1;
  end
% end
