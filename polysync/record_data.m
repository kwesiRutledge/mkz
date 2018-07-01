% record_data.m

%% Constants

subscribers = {};
message_strs = {'ByteArrayMessage', ... 		%1
				'CanFrameMessage', ...			%2
				'CommandMessage' , ...			%3
				'DiagnosticTraceMessage', ...	%4
				'EventMessage', ...				%5
				'FileExtMessage', ...			%6
				'FileMessage',...				%7
				'FileTransferExtMessage', ...	%8
				'FileTransferMessage', ...		%9
				'GpsAccuracyMessage', ...		%10
				'GpsMessage', ...				%11
				'ImageDataMessage' , ...		%12
				'ImuAccuracyMessage', ...		%13
				'ImuMessage', ...				%14
				'LaneModelMessage', ...			%15
				'LidarPointsMessage', ...		%16
				'ManagerStatusMessage', ...		%17
				'ObjectsMessage', ...			%18
				'ParametersMessage', ...		%19
				'PlatformBrakeCommandMessage', ...		%20
				'PlatformBrakeInformationMessage', ...	%21
				'PlatformBrakeReportMessage', ...		%22
				'PlatformCabinExtReportMessage', ...	%23
				'PlatformCabinReportMessage', ...		%24
				'PlatformControlMessage', ...			%25
				'PlatformGearCommandMessage', ...		%26
				'PlatformGearReportMessage', ...		%27
				'PlatformMotionMessage',...				%28
				'PlatformObdMessage', ...				%29
				'PlatformSteeringCommandMessage',...	%30
				'PlatformSteeringReportMessage', ...	%31
				'PlatformSurroundReportMessage',...		%32
				'PlatformSuspensionReportMessage', ...	%33
				'PlatformThrottleCommandMessage', ...	%34
				'PlatformThrottleReportMessage', ...	%35
				'PlatformTirePressureReportMessage', ...%36
				'PlatformTurnSignalCommandMessage', ...	%37
				'PlatformWheelSpeedReportMessage', ...	%38
				'RadarTargetsMessage', ...	%39
				'ResponseMessage', ...		%40
				'RnrMessage',...			%41
				'RnrSessionsMessage', ...	%42
				'SdfStateMessage', ...		%43
				'TrafficSignMessage', ...	%44
				'ZonesMessage'				%45
				};

RTK_SENSOR_ID = 1;    % sensor id of RTK GPS

T_s = 0.01; %Sampling Time for Polysync

%% Select which messages to receive.

consider_msgs = [1:length(message_strs)]';
ignore_msgs = [20;26;30;34;37];

consider_msgs = setdiff(consider_msgs,ignore_msgs);

%% Create Subscribers

ind = 1;
for sub_num = consider_msgs'
	switch(message_strs{sub_num})
	case 'PlatformMotionMessage'
		subscribers{ind} = ...
		eval(['polysync.Subscriber(''MessageType'', ''' message_strs{sub_num} ''',' ...
				'''SensorId'', RTK_SENSOR_ID)' ])
	otherwise	
		subscribers{ind} = ...
			eval(['polysync.Subscriber(''MessageType'', ''' message_strs{sub_num} ''')' ])
	end
	ind = ind+1;
end

%% Begin Saving Data

polyOut_all = [];

DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');
while (ishandle(H))
   	%Save Data to matrices (or cell arrays?)
   	for sub_num = consider_msgs'
   		[~,msg_out] = subscribers{sub_num}.step();
   		polyOut_t{sub_num} = msg_out;
   	end

   	polyOut_all = [polyOut_all; polyOut_t];

   	polysync.Sleep(T_s);
end

c = clock;

save([ date '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'  ],'polyOut_all','consider_msgs')
