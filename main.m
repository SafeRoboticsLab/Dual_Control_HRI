close all; clear all; clc
addpath(genpath('./util'))
addpath('./data')
setenv('SNOPT_LICENSE','/Users/haiminhu/licenses/snopt7.lic'); %**REPLACE**
%
% Main simulation script of Implicit Dual Control for HRI
%
% Author: Haimin Hu (last modified 2022.2.13)


%% Static paramters
load ./data/params
num_players = 2;                % Number of players
ts = 0.2;                       % Sampling time
vDes_H = 12;                    % Human's desired cruising speed                
vDes_R = vDes_H + 3;            % Robot's desired cruising speed
lc = params.rd_bd_max/2;        % Lane center position (lateral)

% Horizons.
N_ilq = 15;                     % iLQGame horizon
N_sim = 80;                     % Maximum simulation horizon
Nd = 2;                         % Number of dual control steps.
Ne = 2;                         % Number of exploitation steps.

% Inital states.
xH_init = [15;   lc; 0.0; 10];
xR_init = [0.0; -lc; 0.0; 10];
x_init  = [xH_init; xR_init];

% Target states.
%   -> Human.
xF_H_l = [0;  1.2; 0; vDes_H; 0; 0; 0; 0]; % Left lane
xF_H_r = [0; -0.8; 0; vDes_H; 0; 0; 0; 0]; % Right lane
uF_H = zeros(2,1);

%   -> Robot.
xF_R = [0; 0; 0; 0; params.xr_tar_overtake; lc; 0; vDes_R];
uF_R = zeros(2,1);

% Control limits.
uH_ub = [2; 0.05];  % Human
uH_lb = -uH_ub;
uR_ub = [4; 0.05];  % Robot
uR_lb = -uR_ub;

uH_constraints.max = uH_ub;
uH_constraints.min = uH_lb;
uR_constraints.max = uR_ub;
uR_constraints.min = uR_lb;

u_constraints_cell = {uH_constraints; uR_constraints};

% Cost matrices.
%   -> Human.
QH = blkdiag(0,5,1,1,0,0,0,0);
RH = blkdiag(0.1,1);

%   -> Robot.
QR_OT = 1*[ 1,0,0,0,-1,0,0,0; zeros(1,8); zeros(1,8); zeros(1,8);
           -1,0,0,0, 1,0,0,0; zeros(1,8); zeros(1,8); zeros(1,8)];
QR_x  = blkdiag(0,0,0,0,0,2,1,1);
QR = QR_OT + QR_x;
RR = blkdiag(0.1,1);

% Covariance of external disturbance d.
Sigma_d = blkdiag(0.1,0.1,0.1,0.1, 0.1,0.1,0.1,0.1);

% Verbose settings.
verbose_iLQ   = false;
verbose_CEMPC = true;
verbose_SMPC  = true;
        

%% iLQGame Setup - Focused Human
% Set up iLQSolvers.
alpha_scaling = 0.01;
max_iteration = 100;
tolerence_percentage = 1e-3;

% Dynamical systems.
Car_H = Bicycle4D(2.7, ts);    % Human (Player 1)
H_id = 1;
Car_R = Bicycle4D(2.7, ts);    % Robot (Player 2)
R_id = 2;
xH_dim = Car_H.x_dim;
xR_dim = Car_R.x_dim;
x_dim = xR_dim + xH_dim;
xH_dims = 1:4;
xR_dims = 5:8;
x_dims = {xH_dims, xR_dims};

uH_dim = Car_H.u_dim;
uR_dim = Car_R.u_dim;
u_dim = uR_dim + uH_dim;
uH_dims = 1:2;
uR_dims = 3:4;
u_dims = {uH_dims, uR_dims};

jointSys = ProductMultiPlayerDynamicalSystem(ts, x_dim, x_dims,...
    u_dim, u_dims, {Car_H, Car_R});

% Initialize Ps and alphas.
Ps_cell = cell(num_players, N_ilq);
alphas_cell = cell(num_players, N_ilq); 
for i = 1:num_players
    for k = 1:N_ilq
        Ps_cell{i,k} = zeros(jointSys.subsystems{i}.u_dim,x_dim);
        alphas_cell{i,k} = zeros(jointSys.subsystems{i}.u_dim,1);                 
    end
end

% -------------------- Set up Human's (Player 1) costs --------------------
% -> Tracking and control cost.
RefDevCost_H_l = ReferenceDeviationCost(xF_H_l, uF_H, QH, RH,...
    "RefDevCost_H_l", x_dim, uH_dim);   % left lane
RefDevCost_H_r = ReferenceDeviationCost(xF_H_r, uF_H, QH, RH,...
    "RefDevCost_H_r", x_dim, uH_dim);   % right lane

% -> Proximity cost.
px_indices = [1,5];
py_indices = [2,6];
max_distance_H = 5;
ProxCost_H = ProductStateProximityCost(px_indices, py_indices,...
    max_distance_H, "ProxCost_H", x_dim, uH_dim);

% -> Lane boundary cost.
pyCost_upper_H = SemiquadraticCost(2, 0.7*params.rd_bd_max, true,...
    'state', "pyCost_upper_H", x_dim, uH_dim);
pyCost_lower_H = SemiquadraticCost(2, 0.7*params.rd_bd_min, false,...
    'state', "pyCost_lower_H", x_dim, uH_dim);

% -> Velocity constraint cost.
vCost_upper_H = SemiquadraticCost(4, vDes_H*1.1, true,  'state',...
    "vCost_upper_H", x_dim, uH_dim);
vCost_lower_H = SemiquadraticCost(4, vDes_R/1.5, false, 'state',...
    "vCost_lower_H", x_dim, uH_dim);

% -------------------- Set up Robot's (Player 2) costs --------------------
% -> Tracking and control cost.
RefDevCost_R = ReferenceDeviationCost(xF_R, uF_R, QR, RR,...
    "RefDevCostF_R", x_dim, uR_dim);

% -> Proximity cost.
max_distance_R = 7;
ProxCost_R = ProductStateProximityCost(px_indices, py_indices,...
    max_distance_R, "ProxCost_R", x_dim, uR_dim);

% -> Lane boundary cost.
pyCost_upper_R = SemiquadraticCost(6, 0.7*params.rd_bd_max, true,...
    'state', "pyCost_upper_R", x_dim, uR_dim);
pyCost_lower_R = SemiquadraticCost(6, 0.7*params.rd_bd_min, false,...
    'state', "pyCost_lower_R", x_dim, uR_dim);

% -> Velocity constraint cost.
vCost_upper_R = SemiquadraticCost(8, vDes_R*1.2, true,  'state',...
    "vCost_upper_R", x_dim, uR_dim);
vCost_lower_R = SemiquadraticCost(8, vDes_R/1.5, false, 'state',...
    "vCost_lower_R", x_dim, uR_dim);

% ----------- Set up (time-varying) total costs for each player -----------
% Build up total costs for H.
Car_H_cost = PlayerCost();
Car_H_cost = Car_H_cost.add_cost(ProxCost_H,       'x',  1e4);
Car_H_cost = Car_H_cost.add_cost(vCost_lower_H,    'x',  4e2);
Car_H_cost = Car_H_cost.add_cost(pyCost_upper_H,   'x',  5e4);
Car_H_cost = Car_H_cost.add_cost(pyCost_lower_H,   'x',  5e4);
Car_H_cost_l = Car_H_cost.add_cost(RefDevCost_H_l, 'xu', 1);
Car_H_cost_r = Car_H_cost.add_cost(RefDevCost_H_r, 'xu', 1);

% Build up total costs for R.
Car_R_cost = PlayerCost();
Car_R_cost = Car_R_cost.add_cost(ProxCost_R,     'x',  1e4);
% Car_R_cost = Car_R_cost.add_cost(vCost_upper_R,  'x',  4e2);
% Car_R_cost = Car_R_cost.add_cost(vCost_lower_R,  'x',  4e2);
Car_R_cost = Car_R_cost.add_cost(pyCost_upper_R, 'x',  5e4);
Car_R_cost = Car_R_cost.add_cost(pyCost_lower_R, 'x',  5e4);
Car_R_cost = Car_R_cost.add_cost(RefDevCost_R,   'xu', 1);

% Set up cost cells.
player_costs_cell_l = cell(num_players, N_ilq);
for k = 1:N_ilq
    player_costs_cell_l{H_id, k} = Car_H_cost_l;
    player_costs_cell_l{R_id, k} = Car_R_cost;
end

player_costs_cell_r = cell(num_players, N_ilq);
for k = 1:N_ilq
    player_costs_cell_r{H_id, k} = Car_H_cost_r;
    player_costs_cell_r{R_id, k} = Car_R_cost;
end


%% iLQGame Setup - Distracted Human
jointSys_DH = ProductMultiPlayerDynamicalSystem(ts, xH_dim, x_dims,...
    uH_dim, u_dims, {Car_H});

% Initialize Ps and alphas.
Ps_cell_EH = cell(1, N_ilq);
alphas_cell_EH = cell(1, N_ilq); 
for k = 1:N_ilq
    Ps_cell_EH{1,k} = zeros(jointSys.subsystems{1}.u_dim, xH_dim);
    alphas_cell_EH{1,k} = zeros(jointSys.subsystems{1}.u_dim, 1);                 
end

% Tracking and control cost.
RefDevCost_EH_l = ReferenceDeviationCost(xF_H_l(xH_dims), uF_H,...
    QH(xH_dims,xH_dims), RH, "RefDevCost_EH_l", xH_dim, uH_dim);
RefDevCost_EH_r = ReferenceDeviationCost(xF_H_r(xH_dims), uF_H,...
    QH(xH_dims,xH_dims), RH, "RefDevCost_EH_r", xH_dim, uH_dim);

% Lane boundary cost.
pyCost_upper_EH = SemiquadraticCost(2, 0.7*params.rd_bd_max, true,...
    'state', "pyCost_upper_EH", xH_dim, uH_dim);
pyCost_lower_EH = SemiquadraticCost(2, 0.7*params.rd_bd_min, false,...
    'state', "pyCost_lower_EH", xH_dim, uH_dim);

% Build up total costs.
Car_H_cost_EH = PlayerCost();
Car_H_cost_EH = Car_H_cost_EH.add_cost(pyCost_upper_EH,   'x',  5e3);
Car_H_cost_EH = Car_H_cost_EH.add_cost(pyCost_lower_EH,   'x',  5e3);
Car_H_cost_EH_l = Car_H_cost_EH.add_cost(RefDevCost_EH_l, 'xu', 1);
Car_H_cost_EH_r = Car_H_cost_EH.add_cost(RefDevCost_EH_r, 'xu', 1);

% Set up cost cells.
player_costs_cell_EH_l = cell(1, N_ilq);
for k = 1:N_ilq
    player_costs_cell_EH_l{1,k} = Car_H_cost_EH_l;
end

player_costs_cell_EH_r = cell(1, N_ilq);
for k = 1:N_ilq
    player_costs_cell_EH_r{1,k} = Car_H_cost_EH_r;
end


%% Main Planning loop

% ===================== Choose your planner =====================
method = 'IDSMPC'; % 'IDSMPC', 'EDSMPC', 'NDSMPC', 'CEMPC', 'iLQ'
% ===============================================================

% Ground-truth human's hidden state.
theta = 1.0;    % [0,1]
M = 'r';        % {'l', 'r'}

% Prior distribution of theta.
mu_theta_init = [0.5; 0.5];
Sigma_theta_init = 5*eye(2);
theta_distr.l.mu = mu_theta_init;
theta_distr.l.covar = Sigma_theta_init;
theta_distr.r.mu = mu_theta_init;
theta_distr.r.covar = Sigma_theta_init;

% Prior distribution of M.
M_distr = [0.5; 0.5];

% LQR terminal cost design.
[AF_R, BF_R] = Car_R.linearizeDiscrete(xF_R(xR_dims), [0;0]);
[~, PR] = dlqr(AF_R, BF_R, QR(xR_dims,xR_dims), RR);

% Create the planner object.
planner.ts = ts;
planner.vDes_H = vDes_H;
planner.vDes_R = vDes_R;
planner.lc = lc;
planner.params = params;
planner.RH = RH;
planner.alpha_scaling = alpha_scaling;
planner.H_id = H_id;
planner.R_id = R_id;
planner.xH_dim = xH_dim;
planner.xR_dim = xR_dim;
planner.xH_dims = xH_dims;
planner.xR_dims = xR_dims;
planner.uH_dim = uH_dim;
planner.uR_dim = uR_dim;
planner.uH_dims = uH_dims;
planner.uR_dims = uR_dims;
planner.jointSys = jointSys;
planner.M_distr_const = [0.5; 0.5];
planner.M_eps = 0.01;

% MPC settings.
extraArg_CEMPC.w_uH = 1e8;    % human's control consistency weight
extraArg_CEMPC.w_RB_l = 1e6;  % road boundary (linear penalty)
extraArg_CEMPC.w_RB_q = 5e6;  % road boundary (quadratic penalty)
extraArg_CEMPC.w_CA_l = 1e7;  % collision avoidance (linear penalty)
extraArg_CEMPC.w_CA_q = 5e7;  % collision avoidance (quadratic penalty)

extraArg_SMPC.M_branch = 1;      % If M branch
extraArg_SMPC.N_M = min(1, Nd);  % M branching horizon
extraArg_SMPC.M_dual = and(1, extraArg_SMPC.M_branch); % If M dual control
extraArg_SMPC.theta_sample_sn = 0.3; % Sampling parameter of theta

extraArg_SMPC.scale_ellips = 1.2; % Collision avoidance ellipsoid scaling
extraArg_SMPC.w_uH   = extraArg_CEMPC.w_uH;
extraArg_SMPC.w_RB_l = extraArg_CEMPC.w_RB_l;
extraArg_SMPC.w_RB_q = extraArg_CEMPC.w_RB_q;
extraArg_SMPC.w_CA_l = extraArg_CEMPC.w_CA_l;
extraArg_SMPC.w_CA_q = extraArg_CEMPC.w_CA_q;

% Initialize simulation.
x = x_init;
X = x;
U = [];
J_em = 0; % Empirical cost of the robot
THETA = theta_distr; % theta trajectory
M_traj = M_distr; % M trajectory

% Main loop.
for t = 1:N_sim
    
    % Change human's hidden states
    if t >= 10
        theta = 0.0;    % [0,1]
        M = 'l';        % 'l', 'r'
    end
    
    % Check goal reaching.
    if x(5) - x(1) >= params.xr_tar_overtake
        break
    end
    
    % Report simulation time.
    disp(['Simulation time: t = ', num2str(t)])
    
    % ========================== Solve iLQGames ===========================
    ilq_solvers = solveiLQgames(x, xH_dims, jointSys, jointSys_DH,...
        Ps_cell, Ps_cell_EH, alphas_cell, alphas_cell_EH,...
        player_costs_cell_l, player_costs_cell_r,...
        player_costs_cell_EH_l, player_costs_cell_EH_r,...
        u_constraints_cell, alpha_scaling, max_iteration,...
        tolerence_percentage, verbose_iLQ);

    % ============================ Robot Block ============================
    if strcmp(method, 'iLQ')
        % ISA-iLQ.
        if M_distr(1) >= M_distr(2)
            u_iLQ = ilq_solvers.AH_l.best_operating_point.us(:,1);
        else
            u_iLQ = ilq_solvers.AH_r.best_operating_point.us(:,1);
        end
        uR = u_iLQ(uR_dims);
    else
        % CEMPC.
        [uR_ce, xR_ce] = getCEMPCcontrol(x, xF_R, Nd + Ne, QR, RR, PR,...
            uR_constraints, uH_constraints, ilq_solvers, theta_distr,...
            M_distr, planner, extraArg_CEMPC, verbose_CEMPC);

        if strcmp(method, 'CEMPC')
            uR = uR_ce(:,1);
            
        elseif strcmp(method, 'NDSMPC') || strcmp(method, 'IDSMPC') ||...
               strcmp(method, 'EDSMPC')
            
            % Warm-start NDSMPC.
            extraArg_SMPC.uR_ws = uR_ce; 
            extraArg_SMPC.xR_ws = xR_ce;
            extraArg_SMPC.opt_mode = 'ws';
            [uR_ws, ~, ~, stree_ws, flag_ws] = getSMPCcontrol(x, xF_R,...
                theta_distr, M_distr, Nd, Ne, QR, RR, PR, Sigma_d,...
                uR_constraints, uH_constraints, ilq_solvers, planner,...
                verbose_SMPC, extraArg_SMPC);

            if strcmp(method, 'IDSMPC')
                % Implicit Dual SMPC.
                if flag_ws
                    % Warm-start SMPC failed.
                    uR = uR_ce(:,1);
                else
                    % Implicit Dual SMPC.
                    extraArg_SMPC.opt_mode = 'std';
                    extraArg_SMPC.fixed_values = 1;
                    extraArg_SMPC.stree = stree_ws;
                    [uR, ~, ~, stree, ~] = getSMPCcontrol(x, xF_R,...
                        theta_distr, M_distr, Nd, Ne, QR, RR, PR,...
                        Sigma_d, uR_constraints, uH_constraints,...
                        ilq_solvers, planner, verbose_SMPC, extraArg_SMPC);
                end

            elseif strcmp(method, 'EDSMPC')
                % Explicit Dual SMPC.
                if flag_ws
                    % Warm-start SMPC failed.
                    uR = uR_ce(:,1);
                else
                    % Explicit Dual SMPC.
                    extraArg_SMPC.opt_mode = 'std';
                    extraArg_SMPC.fixed_values = 1;
                    extraArg_SMPC.stree = stree_ws;
                    extraArg_SMPC.weight_IG = 1e10; %(1e12)
                    [uR, ~, ~, stree, ~] = getSMPCcontrol(x, xF_R,...
                        theta_distr, M_distr, Nd, Ne, QR, RR, PR,...
                        Sigma_d, uR_constraints, uH_constraints,...
                        ilq_solvers, planner, verbose_SMPC,...
                        extraArg_SMPC);
                end

            elseif strcmp(method, 'NDSMPC')
                % NDSMPC.
                if flag_ws
                    % Warm-start SMPC failed.
                    uR = uR_ce(:,1);
                else
                    uR = uR_ws;
                end
            end
        else
            error('Invalid planner option!')
        end
    end
    
    % ============================ Human Block ============================
    % Human control basis functions.
    if M == 'l'
        u_FH = ilq_solvers.AH_l.best_operating_point.us(:,1);
        u_DH = ilq_solvers.EH_l.best_operating_point.us(:,1);
    elseif M == 'r'
        u_FH = ilq_solvers.AH_r.best_operating_point.us(:,1);
        u_DH = ilq_solvers.EH_r.best_operating_point.us(:,1);
    end
    
    % Rational human action.
    uH_rat = theta*u_FH(uH_dims) + (1-theta)*u_DH(uH_dims); 
    
    % Randomize human action.
    eps_noise = [0.2; 0.005];
    uH = uH_rat + eps_noise.*(1-2*rand(2,1));
    
    % Project uH back to its bound.
	uH = max(min(uH,uH_ub),uH_lb);

    % ========================= Update Simulation =========================
    u = [uH; uR];
    x_next = jointSys.integrate(x, u, ts);
    
    % Check collision.
    xR = x_next(xR_dims);
    xH = x_next(xH_dims);
    coll_flag = is_collision(xR, xH, params);
    if coll_flag
        disp('Collision!')
    end
    
    % ======================== Bayesian Inference =========================
    [A_now, B_cell_now] = jointSys.linearizeDiscrete_B_cell(x, u);
    f_x_now = A_now * x;
    iLQ_result = getiLQresults(x, B_cell_now, ilq_solvers, planner, 1);
    
    % theta inference.
    [mu_theta_l, Sigma_theta_l] = updateTheta('l', theta_distr.l.mu,...
        theta_distr.l.covar, Sigma_d, x_next, u, f_x_now, B_cell_now,...
        iLQ_result, planner, []);
    mu_theta_l = mu_theta_l ./ sum(mu_theta_l);
    theta_distr.l.mu = mu_theta_l;
    theta_distr.l.covar = Sigma_theta_l;
    
    [mu_theta_r, Sigma_theta_r] = updateTheta('r', theta_distr.r.mu,...
        theta_distr.r.covar, Sigma_d, x_next, u, f_x_now, B_cell_now,...
        iLQ_result, planner, []);
    mu_theta_r = mu_theta_r ./ sum(mu_theta_r);
    theta_distr.r.mu = mu_theta_r;
    theta_distr.r.covar = Sigma_theta_r;
    
    % M inference.
    extraArg_updateM.scale = 5e2;
    M_distr = updateM(M_distr, theta_distr, x_next, uR, f_x_now,...
        B_cell_now, iLQ_result, planner, extraArg_updateM);
    
    % Store results.
    X = [X x_next];
    U = [U u];
    THETA = [THETA theta_distr];
    M_traj = [M_traj M_distr];
    
    % Shift current state.
    x = x_next;
    
    % Update the empirical cost.
    J_em = updateEmpiricalCost(J_em, x, xF_R, uR, QR, RR);
end


disp('*******************************************************')
disp(['Simulation finished! Empirical cost = ', num2str(J_em)])
disp('*******************************************************')


%% Visualization
XH = X(xH_dims,:);
UH = U(uH_dims,:);
XR = X(xR_dims,:);
UR = U(uR_dims,:);

option.coordinate = 'rel';   % 'rel', 'abs'
option.keep_traj  = 1;
option.is_fading  = 1;
option.t_skip     = 1;
option.t_start    = [];
option.t_end      = [];
option.pause      = 0.0;
option.N_interp   = 2;
option.UI         = 0;
plotSim(planner, XR, XH, THETA, M_traj, option)


%% Auxiliary function
function flag = is_collision(xR, xH, params)
    flag = false;
    % Check collision with the human.
    if (abs(xR(1)-xH(1)) <= 3) && (abs(xR(2)-xH(2)) <= 1)
        flag = true;
    end
    % Check driving out of lanes.
    if (xR(2) >= params.rd_bd_max) || (xR(2) <= params.rd_bd_min)
        flag = true;
    end
end