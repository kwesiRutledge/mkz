function [ u , sol_info ] = tiancheng_mpc( x0 , u0 , steer0 , rd , t_horizon , dT , mpc_wghts )
	% Description:
	%	Given the current state (x0), the MPC methods that Tiancheng proposed
	%	are applied to generate the next steering input for the Lane Keeping
	%	system.
	%
	% Assumptions:
	%	- Lane Keeping Model assumed
	%	- The following remains constant during planned path:
	%		+ Speed
	%		+ Curvature (r_d)
	%	- We want a reduction in state as defined by 'range_tg'
	%
	% Inputs:
	%	- x0: The current (or initial) state for MPC.
	%			[ 	y 		]	lateral deviation
	%		x = [ 	v 		]	lateral speed
	%			[ Delta psi ]	yaw angle
	%			[ 	r 		]	yaw rate
	%	- u0: The Current speed.
	%	- steer0: The previous steering input
	%	- rd: Road curvature. 
	%		+ scalar value
	%		+ In Tiancheng's original code, this was selected by looking
	%		  at a point 0.5 m ahead and assuming that the curvature there
	%		  was constant throughout the time horizon 1:T
	%	- t_horizon: Planning horizon.
	%	- dT: Sampling frequency
	%	- mpc_wghts: The weighting matrices for the MPC problem.
	%		+ For now, this will be a struct with members: Wy,Wv,Wp,Wr,Ws


	%%%%%%%%%%%%%%%
	%% Constants %%
	%%%%%%%%%%%%%%%

	a = 0.17;
	b = 0.16;

	Caf = 14.66;
	Car = Caf*a/b;
	Iz = 0.0833/2.5;

	%Range variable (limits the magnitude of future states)
	range_tg = [0.15; 0.5; 10/180*pi; 20/180*pi; 200*atan(12.5/147)/70];

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	%% Initializing Matrices

	A = [0, 1, u0, 0;
	     0, -(Caf + Car)/u0, 0, ((-a*Caf + b*Car)/u0 - u0);
	     0, 0, 0, 1;
	     0, (-a*Caf + b*Car)/(Iz*u0), 0, -(a^2*Caf + b^2*Car)/(Iz*u0)];
	B = [0; Caf; 0; a*Caf/Iz];
	E = [0; 0; -1; 0];
	K = [0; 0; 0; 0];

	x0 = [x0', steer0 ];

	%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Creating MPC Matrices %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%

	[Aeq, Beq] = dyn2AeqBeq(A, B, E*rd, K, x0, t_horizon, dT);
	%                 unitMatA = [zeros(1, S+1); eye(t_horizon), zeros(t_horizon, 1)];
	%                 unitMatB = blkdiag(0, eye(t_horizon));
	%                 Aeq = kron(-Ad, unitMatA) + kron(-repB, unitMatB) + kron(blkdiag(eye(4),0), eye(t_horizon+1)) + kron(blkdiag(zeros(4,4),1), blkdiag(1,zeros(t_horizon,S)));
	%                 Beq = kron([Ed;0], [0; ones(S,1)]) + kron(x0', [1; zeros(t_horizon,1)]);

	Aueq = kron(eye(5), [-eye(t_horizon+1); eye(t_horizon+1)]);
	Bueq = kron(range_tg, repmat([0; ones(t_horizon,1)], 2, 1)) + kron(1000*ones(5,1), repmat([1; zeros(t_horizon,1)],2,1));

	%% weight matrix for input
	iWMat = 2*eye(t_horizon+1)...
	        + [zeros(t_horizon,1), -eye(t_horizon);zeros(1,t_horizon+1)]...
	        + [zeros(t_horizon,1), -eye(t_horizon);zeros(1,t_horizon+1)]';
	iWMat(1) = 1;
	iWMat(end) = 1;

	H = 2*blkdiag(mpc_wghts.Wy*eye(t_horizon+1),...
	              mpc_wghts.Wv*eye(t_horizon+1),...
	              mpc_wghts.Wp*eye(t_horizon+1),...
	              mpc_wghts.Wr*eye(t_horizon+1),...
	              mpc_wghts.Ws*iWMat);
	          
	f = kron([0; -2*mpc_wghts.Wv*rd; 0; -2*mpc_wghts.Wr*rd; 0], ones(t_horizon+1, 1));
	%                 H = 2*blkdiag(kron([obj.Wy, 0, -obj.Wy*obj.L;
	%                                     0, obj.Wv, obj.Wv*u;
	%                                     -obj.Wy*obj.L, obj.Wv*u, obj.Wv*u^2+obj.Wp+obj.Wy*obj.L^2], eye(t_horizon+1)),...
	%                               (obj.Wr)*eye(t_horizon+1),...
	%                               obj.Ws*iWMat);

	%                 f = kron([0; 0; 0; -2*obj.Wr*rd; 0], ones(S+1, 1));
	% disp('LK_sys')
	
	silent_options = optimoptions('quadprog','Display','none');

	[optx,fval,exitflag,output] = quadprog(H,f,Aueq,Bueq,Aeq,Beq,[],[],[],silent_options);

	%%%%%%%%%%%%%%%%%%%%
	%% Saving Results %%
	%%%%%%%%%%%%%%%%%%%%

	u = optx(end-t_horizon+1:end);

	sol_info.optx = optx;
	sol_info.fval = fval;
	sol_info.quadprog_exitflag = exitflag;
	sol_info.output = output;
end	
