clear all
close all

save_movies = 0;

if save_movies
	vidObj = VideoWriter('take over.mp4', 'MPEG-4');
	vidObj.FrameRate = 10;
	open(vidObj);
else
    vidObj = 0;
end

road1 = (0:0.01:100)';
road1 = [road1, zeros(length(road1), 1)];
curv1 = zeros(length(road1), 1);
road2 = road1;
road2(:,2) = road2(:,2) + 0.4;
curv2 = zeros(length(road1), 1);

uL = 0.3;
XL = 2;

dT = 0.1;
X = 0;
Y = 0;
psi = 0;
u = 0.3;
h = XL - X;
v = 0;
r = 0;
lat_control = get_lat_input;
ACC_control = get_ACC_input;
CC_control = get_CC_input;
% CC_control.setup(0.8);
I = 0;
steer = 0;

mode = 1;

uData = [];
IData = [];
hData = [];

for t = 0:dT:20
    if t <= 5
        mode = 1;
    elseif t > 5 && t <= 10
        mode = 2;
    elseif t > 10 && t <= 15
        mode = 3;
    elseif t > 15 && t <= 20
        mode = 4;
    end
    
    if (mode == 1 || mode == 4)
        road = road1;
        curv = curv1;
    else
        road = road2;
        curv = curv2;
    end

    intersectL = intersectLine(X, Y, psi);
    intersect_ind = InterX(road', intersectL);
    intersect_ind = intersect_ind(1);
    if (size(intersect_ind,2)>1)
        error('Multiple line intersect')
    end
    if (isempty(intersect_ind))
        error('Road out of view')
    end
    intersect = road(intersect_ind,1:2);
    plot(road(:,1), road(:,2))
    axis equal
    xlim([0,10])
    ylim([-2.5,2.5])
    hold on
    plot_car(X, Y, psi, 'r')
    plot_car(XL, 0, 0, 'g')
    plot(intersectL(1,:),intersectL(2,:))
    plot(intersect(1),intersect(2),'or')
    drawnow
    set(gcf,'color','w');
    if save_movies
        axis off
        currFrame = getframe(gca, [0 0 430 344]);
        writeVideo(vidObj, currFrame);
    end
    hold off
    
    if mode == 1 || mode == 2
        h = XL-X-0.7;
        I = ACC_control.step(u,h,uL/cos(psi));
        if mode == 2 && abs(y) < 0.01
            mode = 3;
        end
    else
        h = X - XL - 0.7;
        I = CC_control.step(u);
        if mode == 3 && h > 0
            mode = 4;
        end
    end
    
    y = get_signed_distance(X, Y, psi, intersect(1),intersect(2))
    p = [X+0.5*cos(psi), Y+0.5*sin(psi)];
    
    rpsi = atan2(road(intersect_ind+1,2)-road(intersect_ind,2), road(intersect_ind+1,1)-road(intersect_ind,1));
    delta_psi = mod(rpsi-psi,2*pi);
    if delta_psi > pi
        delta_psi = delta_psi - 2*pi;
    end
    delta_psi
    rd = u*curv(intersect_ind);
    steer = lat_control.step(u, y, v, delta_psi, r, rd);
    [X, Y, psi, u, v, r] = find_nextState(X, Y, psi, u, v, r, I, steer, dT);
    
    XL = XL+uL*dT;
    uData = [uData, u];
    IData = [IData, I];
    hData = [hData, h];
end

if save_movies
	close(vidObj)
end

figure
plot(0:dT:20, uData)
title('u')

figure
plot(0:dT:20, IData)
title('I')

figure
plot(0:dT:20, hData)
title('h')