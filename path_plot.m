%% path_plot: plot the path stored in a road object
function [] = path_plot(varargin)
	%Description:
	%	
	%Input Format:
	%	path_plot(file) 						%Here for historical reasons
	%	path_plot(filename,pathname)
	%	path_plot(filename,pathname,curr_dir)

	path_to_mkz = [];

	switch nargin
	case 1
		disp('Idk what this means.')
		[filename, pathname] = uigetfile;
	case 2
		filename = varargin{1};
		pathname = varargin{2};
	case 3
		filename = varargin{1};
		pathname = varargin{2};
		path_to_mkz = varargin{3};
	end

	r = road;
	r.pathfile = strcat(pathname, filename);
	r.setup(struct('latitude', 0, 'longitude', 0));

	disp('Nominal.')

	s_vec = 0:0.5:r.len_path;

	[rc, drc, kappa] = arrayfun(@(s) r.get_pos(s), s_vec, ...
						  		'UniformOutput', false);
	rc = cell2mat(rc);
	drc = cell2mat(drc)';
	kappa = cell2mat(kappa);

	tt = 0.0001*[-drc(:,2) drc(:,1)]./sqrt(sum(drc.*drc, 2));

	[lat, long, h] = enu2geodetic(rc(1,:), rc(2,:), 0, r.lat0, r.long0, r.h0, wgs84Ellipsoid);

	figure(1); clf; 
	hold on;
	mapshow([ path_to_mkz 'mcity/mcity.tiff']);
	plot(long, lat);
	for i=1:length(long)
		plot([long(i) long(i)+kappa(i)*tt(i,1)], [lat(i) lat(i)+kappa(i)*tt(i,2)], 'c')
	end
	% plot(xr, yr, '*');
	figure(2); clf;
	plot(s_vec, kappa);
	ylim([-0.1 0.1])
end