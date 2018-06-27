clear all
close all

%Recording Video, if desired;
save_movies = 1;

if save_movies
	vidObj = VideoWriter('lane keeping.mp4', 'MPEG-4');
	vidObj.FrameRate = 10;
	open(vidObj);
else
    vidObj = 0;
end

%Defining Constants like
%   - Maximum Road Curvature
%   - Maximum allowable input
%   - Simulation Start and end times
%   - road shape (just a pairing of x-y coords)
%   - curvature of the road from each point in road shape
rd_max = pi/12;
u_max = 0.5;
disp(['rho=',num2str(u_max/rd_max)])
t = [0, 20];

road = generate_road_sin(t, rd_max, u_max);
curv = -LineCurvature2D(road);

dT = 0.1;
%Initial conditions
X = 0;
Y = 0;
psi = 0;
ur = 0.5; % desired longitudinal velocity (u reference)
u = 0.5; % longitudinal velocity
v = 0;
r = 0;
% Initialize the controller
lat_control = get_lat_input;
CC_control = get_CC_input;
I = 0;

for t = 0:dT:20
    intersectL = intersectLine(X, Y, psi);      %No way to describe this function other than that it defines a line
                                                %(using 2 points, 1 in each column of intersectL) that is
                                                %perpendicular to the current direction of travel (psi) and 0.5 units
                                                %of distance in front of the car (in front of X,Y)

    intersect_ind = InterX(road', intersectL);  %Returns the indices of road at which intersectL is the closest when
                                                %the 2 intersect. If there is no intersection, then the result is an
                                                %empty matrix.
    if (size(intersect_ind,2)>1)
        error('Multiple line intersect')
    end
    if (isempty(intersect_ind))
        error('Road out of view')
    end
    intersect = road(intersect_ind,1:2);
    plot(road(:,1), road(:,2))
    axis equal
    hold on
    plot_car(X, Y, psi, 'r')
    plot(intersectL(1,:),intersectL(2,:))       %Drawing the line that intersects with the road
    plot(intersect(1),intersect(2),'or')
    drawnow
    set(gcf,'color','w');
    if save_movies
        axis off
        currFrame = getframe(gca, [0 0 430 344]);
        writeVideo(vidObj, currFrame);
    end
    hold off
    
    y = get_signed_distance(X, Y, psi, intersect(1),intersect(2));
    p = [X+0.5*cos(psi), Y+0.5*sin(psi)];
    
    % road psi
    rpsi = atan2(road(intersect_ind+1,2)-road(intersect_ind,2), road(intersect_ind+1,1)-road(intersect_ind,1));
    delta_psi = mod(rpsi-psi,2*pi);
    if delta_psi > pi
        delta_psi = delta_psi - 2*pi;
    end
    rd = u*curv(intersect_ind);
    I = CC_control.step(u, ur);
    steer = lat_control.step(u, y, v, delta_psi, r, rd);
    [X, Y, psi, u, v, r] = find_nextState(X, Y, psi, u, v, r, I, steer, dT);
end

if save_movies
	close(vidObj)
end