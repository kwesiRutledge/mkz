filename = uigetfile;

data_ = load(filename);

tmin = 0;
tmax = Inf;

tidx = and(data_.gpsdata.Longitude.Time < tmax, ...
		   tmin < data_.gpsdata.Longitude.Time);

tidx_gps = and(tidx, squeeze(data_.gpsdata.Longitude.Data) ~= 0);

figure(1); clf
plot(squeeze(data_.gpsdata.Longitude.Data(tidx_gps)), ...
	 squeeze(data_.gpsdata.Latitude.Data(tidx_gps)))

figure(2)
clf; hold on;
subplot(311)
hold on
plot(data_.lk_acc_state.y.Time(tidx), data_.lk_acc_state.y.Data(tidx))
ylim([-1 1]);
ylabel('y')

subplot(312)
hold on
plot(data_.lk_acc_state.nu.Time(tidx), data_.lk_acc_state.nu.Data(tidx))
ylabel('nu')

subplot(313)
hold on
plot(data_.lk_acc_state.dPsi.Time(tidx), data_.lk_acc_state.dPsi.Data(tidx))
ylim([-0.5 0.5]);
ylabel('\Delta \Psi')

figure(3); clf
subplot(211)
hold on
plot(data_.lk_acc_state.r.Time(tidx), data_.lk_acc_state.r.Data(tidx))
ylim([-1 1])
ylabel('r')

subplot(212)
hold on 
plot(data_.lk_acc_state.mu.Time(tidx), data_.lk_acc_state.mu.Data(tidx))
ylabel('mu')
ylim([0, 10])


NUM = 70;   % number of bytes to read
string_vec = char(squeeze(data_.ba_message.Bytes.Data(1:NUM,1,1:end))');
string_cell = mat2cell(string_vec, ones(1, size(string_vec,1)), NUM);

pattern = 'OPENCAV:BRAKE%f,THROTTLE%f,STEER%f,GEAR%d,SIGNAL%d,END';

ba_data_cell = cellfun(@(s) sscanf(s, pattern), string_cell, 'UniformOutput', false);

ba_data_vec = zeros(5, length(ba_data_cell));
ba_data_vec(:, ~cellfun(@isempty,ba_data_cell)) = cell2mat(ba_data_cell');

brake = ba_data_vec(1,:);
throttle = ba_data_vec(2,:);
steering = ba_data_vec(3,:);

figure(4); clf
subplot(211)
plot(data_.ba_message.Bytes.Time(tidx), throttle(tidx))
xlabel('Time')
ylabel('Throttle Value')
title('Throttle Trajectory')

subplot(212)
plot(data_.ba_message.Bytes.Time(tidx), steering(tidx))
xlabel('Time')
ylabel('Steering Value')
title('Steering Trajectory')
