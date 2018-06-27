clear all
close all

% Description:
%   This script is meant to compare the multiple MPC algorithms suggested for the Lane Keeping
%   part of the MCity MKZ project.
%   The 3 Methods are:
%       1. A variant from Tiancheng Ge (get_lat_input)
%       2. Petter Nilsson's Old MPC Algorithm (Greedily looks at the next step only)
%       3. My Own
%   This script and this example were created from example code provided by Tiancheng Ge. So some
%   unused parts may be commented out.

%Recording Video, if desired;
% save_movies = 1;

% if save_movies
% 	vidObj = VideoWriter('lane keeping.mp4', 'MPEG-4');
% 	vidObj.FrameRate = 10;
% 	open(vidObj);
% else
%     vidObj = 0;
% end

%%%%%%%%%%%%%%%
%% Functions %%
%%%%%%%%%%%%%%%

%Import functions
if isempty(strfind(path,'../systems/'))
    addpath('../systems/')
end

if isempty(strfind(path,'./functions/'))
    addpath('./functions/')
end

if isempty(strfind(path,'./functions/simulator/'))
    addpath('./functions/simulator/')
end

%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

%Defining Constants like
%   - Maximum Road Curvature
%   - Maximum allowable input
%   - Simulation Start and end times
%   - road shape (just a pairing of x-y coords)
%   - curvature of the road from each point in road shape
rd_max = pi/12;
u_max = 0.5;    %Maximum speed for the car?
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

% Initialize the Controllers
lat_control = get_lat_input;
CC_control = get_CC_input;
I = 0;

%Tiancheng's Controller Class Parameters
S = 20; %Desired Time Horizon
N = 20;     %No longer used. Same as desired time horizon
Wy = 10;    %Weight for the Y state
Wv = 10;
Wp = 10;
Wr = 20;
Ws = 10;

%Range variable (limits the magnitude of future states)
range_tg = [0.15; 0.5; 10/180*pi; 20/180*pi; 200*atan(12.5/147)/70];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Applying the Controller's to the initial condition %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Tiancheng
intersectL = intersectLine(X, Y, psi);
intersect_ind = InterX(road', intersectL);

if (size(intersect_ind,2)>1)
    error('Multiple line intersect')
end
if (isempty(intersect_ind))
    error('Road out of view')
end
intersect = road(intersect_ind,1:2);

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
% [X, Y, psi, u, v, r] = find_nextState(X, Y, psi, u, v, r, I, steer, dT);

% Duplicating Tiancheng

steer2 = 0;    %Steering initial condition
            %At the initial state, Tiancheng's starts from 0,
            %but after it uses the previous steering command.

%% Initializing Matrices
% dt = obj.T;

a = 0.17;
b = 0.16;

Caf = 14.66;
Car = Caf*a/b;
Iz = 0.0833/2.5;

A = [0, 1, u, 0;
     0, -(Caf + Car)/u, 0, ((-a*Caf + b*Car)/u - u);
     0, 0, 0, 1;
     0, (-a*Caf + b*Car)/(Iz*u), 0, -(a^2*Caf + b^2*Car)/(Iz*u)];
B = [0; Caf; 0; a*Caf/Iz];
E = [0; 0; -1; 0];
K = [0; 0; 0; 0];

%                 A_s = @(s) expm(s*A);
%                 Ad = A_s(dt);
%                 integral_A = integral(A_s, 0, dt, 'ArrayValued', true);
%                 Bd = integral_A * B;
%                 Ed = integral_A * E * rd;
% 
%                 Ad = blkdiag(Ad, 0);
%                 repB = zeros(5,5);
%                 repB(1:4, 5) = Bd;

x0 = [y, v, delta_psi, r, steer2 ];
[Aeq, Beq] = dyn2AeqBeq(A, B, E*rd, K, x0, S, dT);
%                 unitMatA = [zeros(1, S+1); eye(S), zeros(S, 1)];
%                 unitMatB = blkdiag(0, eye(S));
%                 Aeq = kron(-Ad, unitMatA) + kron(-repB, unitMatB) + kron(blkdiag(eye(4),0), eye(S+1)) + kron(blkdiag(zeros(4,4),1), blkdiag(1,zeros(S,S)));
%                 Beq = kron([Ed;0], [0; ones(S,1)]) + kron(x0', [1; zeros(S,1)]);

Aueq = kron(eye(5), [-eye(S+1); eye(S+1)]);
Bueq = kron(range_tg, repmat([0; ones(S,1)], 2, 1)) + kron(1000*ones(5,1), repmat([1; zeros(S,1)],2,1));

%% weight matrix for input
iWMat = 2*eye(S+1)...
        + [zeros(S,1), -eye(S);zeros(1,S+1)]...
        + [zeros(S,1), -eye(S);zeros(1,S+1)]';
iWMat(1) = 1;
iWMat(end) = 1;

H = 2*blkdiag(Wy*eye(S+1),...
              Wv*eye(S+1),...
              Wp*eye(S+1),...
              Wr*eye(S+1),...
              Ws*iWMat);
          
f = kron([0; -2*Wv*rd; 0; -2*Wr*rd; 0], ones(S+1, 1));
%                 H = 2*blkdiag(kron([obj.Wy, 0, -obj.Wy*obj.L;
%                                     0, obj.Wv, obj.Wv*u;
%                                     -obj.Wy*obj.L, obj.Wv*u, obj.Wv*u^2+obj.Wp+obj.Wy*obj.L^2], eye(S+1)),...
%                               (obj.Wr)*eye(S+1),...
%                               obj.Ws*iWMat);

%                 f = kron([0; 0; 0; -2*obj.Wr*rd; 0], ones(S+1, 1));
disp('LK_sys')
[optx,fval,exitflag,output] = quadprog(H,f,Aueq,Bueq,Aeq,Beq);

%% Use Function to Get the Same Results
W_cont.Wy = Wy;
W_cont.Wv = Wv;
W_cont.Wp = Wp;
W_cont.Wr = Wr;
W_cont.Ws = Ws;

[u,fcn_info] = tiancheng_mpc( [ y;v;delta_psi;r ] , u , steer2 , rd , S , dT , W_cont )

%% Compare Results

disp('=======')
disp('Results')
disp(['lat_control.step(u, y, v, delta_psi, r, rd): ' num2str(steer) ])
disp(['optx(end-S+1): ' num2str(optx(end-S+1)) ])
disp(['Function: ' num2str(u(1)) ])

exp2_data.lat_control_step = steer;
exp2_data.optx_raw = optx(end-S+1);
exp2_data.fcn.u = u;
exp2_data.fcn.fcn_info = fcn_info;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Experiment 2: Comparing the selected u from different mpc methods %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('========================================================')
disp('Experiment 2')
disp('+ Comparing all 3 methods on the current sinusoidal road')
disp('+ At the starting point, what input is being suggested ')
disp('+ Visualize why it was selected')
disp(' ')

%Initial conditions
X = 0;
Y = 0;
psi = 0;
ur = 0.5; % desired longitudinal velocity (u reference)
u = 0.5; % longitudinal velocity
v = 0;
r = 0;

disp('Initial Conditions:')
disp('+ X = 0')
disp('+ Y = 0')
disp('+ psi = 0')
disp('+ ur = 0.5')
disp('+ u = 0.5')
disp('+ v = 0')
disp('+ r = 0')
disp(' ')

disp('Results:')

% Tiancheng's suggestion is already given. Let's try to plot the future points
%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

x_tg = struct;
for i = 1 : S
    [ x_tg(i).x , x_tg(i).y , x_tg(i).psi , x_tg(i).u , x_tg(i).v , x_tg(i).r ] = find_nextState(X, Y, psi, u, v, r, I, exp2_data.fcn.u(i), dT);

    X   = x_tg(i).x;
    Y   = x_tg(i).y;
    psi = x_tg(i).psi;
    u   = x_tg(i).u;
    v   = x_tg(i).v;
    r   = x_tg(i).r;

end

figure;
temp = subplot(1,2,1)
hold on;
for i = 1:S
    plot_car(x_tg(i).x,x_tg(i).y,x_tg(i).psi,x_tg(i).r)
end
title('Predicted Course of TG''s Car')

subplot(1,2,2)

hold on;
plot(road(:,1), road(:,2))
plot([x_tg(:).x],[x_tg(:).y])
plot(intersectL(1,:),intersectL(2,:))       %Drawing the line that intersects with the road
plot(intersect(1),intersect(2),'or')

axis([ temp.XLim temp.YLim ])
title('Planned Trajectory vs. Road')
legend('Road','Planned Trajectory','\kappa Meas. Here')

% My Simple MPC
%++++++++++++++

%Initial conditions
X = 0;
Y = 0;
psi = 0;
ur = 0.5; % desired longitudinal velocity (u reference)
u = 0.5; % longitudinal velocity
v = 0;
r = 0;

%Get Discretized System

A_s = @(s) expm(s*A);
Ad = A_s(dT);
integral_A = integral(A_s, 0, dT, 'ArrayValued', true);
Bd = integral_A * B;
Ed = integral_A * E;

lk_sys.A    = Ad;
lk_sys.B    = Bd;
lk_sys.C    = eye(4);  %Not used. Given a garbage value.
%Need to create the initial condition carefully.
%y is not truly known. Needs to be calculated using the signed_distance function.
lk_sys.x0   = [ 0 ; %get_signed_distance( 0 , 0, psi, intersect(1),intersect(2));
                0 ;
                0 ;
                0 ];

lk_sys.E = Ed;

[ G,H,C_big,x0_m ] = create_skaf_n_boyd_matrices( lk_sys , S );

one_vec = ones(S,1);

% Use MPC QP Solver

H_0 = one_vec'*H'*H*one_vec + 2;
f_0 = one_vec'*H'*(G*kron(eye(S),lk_sys.E)*one_vec*rd + x0_m);

L = chol(H_0,'lower');
Linv = L\eye(size(H_0,1));

[ u_mpc2 , mpc2_status ] = mpcqpsolver( Linv , f_0 , [] , zeros(0,1) , [] , zeros(0,1) , false(0,1) , mpcqpsolverOptions() )

%Apply to system

mpc2.state = struct;
for i = 1 : S
    [ mpc2.state(i).x , mpc2.state(i).y , mpc2.state(i).psi , mpc2.state(i).u , mpc2.state(i).v , mpc2.state(i).r ] = find_nextState(X, Y, psi, u, v, r, I, u_mpc2, dT);

    X   = mpc2.state(i).x;
    Y   = mpc2.state(i).y;
    psi = mpc2.state(i).psi;
    u   = mpc2.state(i).u;
    v   = mpc2.state(i).v;
    r   = mpc2.state(i).r;

end

figure;

hold on;
plot(road(:,1), road(:,2))
plot([mpc2.state(:).x],[mpc2.state(:).y])
plot(intersectL(1,:),intersectL(2,:))       %Drawing the line that intersects with the road
plot(intersect(1),intersect(2),'or')

axis([ temp.XLim temp.YLim ])
title('Planned Trajectory vs. Road')
legend('Road','Planned Trajectory','\kappa Meas. Here')

% for t = 0:dT:20
%     intersectL = intersectLine(X, Y, psi);      %No way to describe this function other than that it defines a line
%                                                 %(using 2 points, 1 in each column of intersectL) that is
%                                                 %perpendicular to the current direction of travel (psi) and 0.5 units
%                                                 %of distance in front of the car (in front of X,Y)

%     intersect_ind = InterX(road', intersectL);  %Returns the indices of road at which intersectL is the closest when
%                                                 %the 2 intersect. If there is no intersection, then the result is an
%                                                 %empty matrix.
%     if (size(intersect_ind,2)>1)
%         error('Multiple line intersect')
%     end
%     if (isempty(intersect_ind))
%         error('Road out of view')
%     end
%     intersect = road(intersect_ind,1:2);
%     plot(road(:,1), road(:,2))
%     axis equal
%     hold on
%     plot_car(X, Y, psi, 'r')
%     plot(intersectL(1,:),intersectL(2,:))       %Drawing the line that intersects with the road
%     plot(intersect(1),intersect(2),'or')
%     drawnow
%     set(gcf,'color','w');
%     if save_movies
%         axis off
%         currFrame = getframe(gca, [0 0 430 344]);
%         writeVideo(vidObj, currFrame);
%     end
%     hold off
    
%     y = get_signed_distance(X, Y, psi, intersect(1),intersect(2));
%     p = [X+0.5*cos(psi), Y+0.5*sin(psi)];
    
%     % road psi
%     rpsi = atan2(road(intersect_ind+1,2)-road(intersect_ind,2), road(intersect_ind+1,1)-road(intersect_ind,1));
%     delta_psi = mod(rpsi-psi,2*pi);
%     if delta_psi > pi
%         delta_psi = delta_psi - 2*pi;
%     end
%     rd = u*curv(intersect_ind);
%     I = CC_control.step(u, ur);
%     steer = lat_control.step(u, y, v, delta_psi, r, rd);
%     [X, Y, psi, u, v, r] = find_nextState(X, Y, psi, u, v, r, I, steer, dT);
% end

% if save_movies
% 	close(vidObj)
% end