[filename, pathname] = uigetfile;

load(strcat(pathname, '/', filename))

% tmin = 15;
% tmax = 25;
tmin = 0;
tmax = Inf;

tidx = find(and(rawdata.longitude.Time < tmax, ...
				tmin < rawdata.longitude.Time));

% Convert BA message
is_ba = exist('ba_message');

if is_ba
	NUM = 70;   % number of bytes to read
	string_vec = char(squeeze(ba_message.Bytes.Data(1:NUM,1,1:end))');
	string_cell = mat2cell(string_vec, ones(1, size(string_vec,1)), NUM);

	pattern = 'OPENCAV:BRAKE%f,THROTTLE%f,STEER%f,GEAR%d,SIGNAL%d,END';

	ddd = cellfun(@(s) sscanf(s, pattern), string_cell, 'UniformOutput', false);
	ddd = cell2mat(ddd');

	brake = ddd(1,:);
	throttle = ddd(2,:);
	steering = ddd(3,:);

	figure(4); clf
	subplot(211)
	plot(throttle)
	subplot(212)
	plot(steering)
end

figure(1); clf
plot(rawdata.longitude.Data(tidx), rawdata.latitude.Data(tidx))

figure(2)
clf; hold on;
subplot(311)
hold on
plot(lk_acc_state.y.Time(tidx), lk_acc_state.y.Data(tidx))
ylim([-1 1]);
ylabel('y')

subplot(312)
hold on
plot(lk_acc_state.nu.Time(tidx), lk_acc_state.nu.Data(tidx))
ylabel('nu')

subplot(313)
hold on
plot(lk_acc_state.dPsi.Time(tidx), lk_acc_state.dPsi.Data(tidx))
ylim([-0.5 0.5]);
ylabel('\Delta \Psi')

figure(3); clf
subplot(311)
hold on
plot(lk_acc_state.r.Time(tidx), lk_acc_state.r.Data(tidx))
ylim([-1 1])
ylabel('r')

subplot(312)
hold on 
plot(lk_acc_state.mu.Time(tidx), lk_acc_state.mu.Data(tidx))
ylabel('mu')
ylim([0, 10])

subplot(313)
hold on 
plot(lk_acc_state.r_d.Time(tidx), lk_acc_state.r_d.Data(tidx))
ylabel('r_d')


% figure(4); clf
% if exist('delta_f')
% 	idx = find(and(delta_f.Time < tmax, delta_f.Time > tmin))
% 	subplot(211)
% 	plot(delta_f.Time(idx), squeeze(delta_f.Data(1,1,idx)))
% 	subplot(212); hold on
% 	plot(lk_cinfo.barrier_val.Time(idx), squeeze(lk_cinfo.barrier_val.Data(idx)))
% 	plot(xlim, [0 0], '-g')
% end