function [xH_Sol, uH_Sol, VSol, status] = getCEMPChumancontrol(x0, xR,...
    xF_H, N, QH, RH, PH, uH_constraints, scale_ellips, jointSys,...
    planner, extraArg, verbose)
%   Nominal Certainty-Equivalent MPC planner that produces the human's
%   controls as a response to the robot's plans.
%
% Author: Haimin Hu (last modified 2022.12.24)

    yalmip('clear')
    params = planner.params;
    
    % Systems
    ts = jointSys.ts;
    H_id = planner.H_id;
    subsys_H = jointSys.subsystems{H_id};
    xH_dim = subsys_H.x_dim;
    uH_dim = subsys_H.u_dim;
    xH_dims = jointSys.x_dims{H_id};

    % Soft constraint weights
    w_RB_l = extraArg.w_RB_l;	% road boundary (linear penalty)
    w_RB_q = extraArg.w_RB_q;	% road boundary (quadratic penalty)
    w_CA_l = extraArg.w_CA_l;	% collision avoidance (linear penalty)
    w_CA_q = extraArg.w_CA_q;	% collision avoidance (quadratic penalty)
    
    % Define solver settings
    options = sdpsettings('verbose', 0, 'solver', 'snopt', 'usex0', 1,...
        'cachesolvers', 1);
    options.snopt.Major_feasibility_tolerance = 1e-5;
    options.snopt.Major_optimality_tolerance = 1e-5;
    options.snopt.Minor_feasibility_tolerance = 1e-5;
    options.snopt.Iterations_limit = 4000;
    options.snopt.Major_iterations_limit = 250;
    options.snopt.Minor_iterations_limit = 125;
    options.snopt.Time_limit = 2;

    %----------------------------------------------------------------------
    % Define decision variables
    %----------------------------------------------------------------------
    % States
    xH = sdpvar(xH_dim, N+1);
    
    % Inputs
    uH = sdpvar(uH_dim, N);
    
    % Soft constraint slack variables
    eps_RB_H_min = sdpvar(1, N+1);
    eps_RB_H_max = sdpvar(1, N+1);
    eps_CA_H = sdpvar(1, N+1);

    %----------------------------------------------------------------------
    % Objective function
    %----------------------------------------------------------------------
    % Terminal cost
    objective = (xH(:,N+1) - xF_H)'*PH*(xH(:,N+1) - xF_H);
    
    % Tracking cost for states and controls
    for k = 1:N
        objective = objective + uH(:,k)'*RH*uH(:,k) +...
            (xH(:,k) - xF_H)'*QH*(xH(:,k) - xF_H);
    end
    
    % Soft state constraint penalties
    for k = 1:N+1
        objective = objective + w_RB_l*eps_RB_H_min(k) +...
            w_RB_q*eps_RB_H_min(k)^2;
        objective = objective + w_RB_l*eps_RB_H_max(k) +...
            w_RB_q*eps_RB_H_max(k)^2;
        objective = objective + w_CA_l*eps_CA_H(k) + w_CA_q*eps_CA_H(k)^2;
    end

    %----------------------------------------------------------------------
    % Input constraints
    %----------------------------------------------------------------------
    constraint = [];
    for k = 1:N
        constraint = [constraint...
            uH_constraints.min <= uH(:,k), uH(:,k) <= uH_constraints.max];
    end
    
    %----------------------------------------------------------------------
    % State constraints
    %----------------------------------------------------------------------
    % Collision avoidance ellipsoids
    ellips = ellipsoid(zeros(2,1),...
        blkdiag(scale_ellips*params.avoid.lgt_ub,...
                scale_ellips*params.avoid.lat_bd)^2);
    [~, Q_CA] = double(ellips);
            
    for k = 1:N+1
        % Road boundaries    
        constraint = [constraint...
            params.rd_bd_min - eps_RB_H_min(k) <= xH(2,k),...
            params.rd_bd_max + eps_RB_H_max(k) >= xH(2,k)];
       
        % Collision avoidance
        constraint = [constraint...
            1 - (xR(1:2,k)-xH(1:2,k))'*inv(Q_CA)*...
            (xR(1:2,k)-xH(1:2,k)) <= eps_CA_H(k)]; %#ok<MINV>
    end

    % Slack variables
    constraint = [constraint, eps_RB_H_min >= 0, eps_RB_H_max >= 0,...
        eps_CA_H >= 0];
    
    %----------------------------------------------------------------------
    % Initial state constraints
    %----------------------------------------------------------------------
    constraint = [constraint xH(:,1) == x0(xH_dims)];

    %----------------------------------------------------------------------
    % Dynamics constraints
    %----------------------------------------------------------------------
    for k = 1:N
        constraint = [constraint...
            xH(:,k+1) == subsys_H.integrate(xH(:,k), uH(:,k), ts)];
    end
    
    %----------------------------------------------------------------------
    % Warm-start
    %----------------------------------------------------------------------
%     assign(xH, extraArg.xH_ws)
%     assign(uH, extraArg.uH_ws)

    %----------------------------------------------------------------------
    % Solve the MPC problem
    %----------------------------------------------------------------------
    tic
    status = optimize(constraint, objective, options);
    total_time = toc;
    stime = status.solvertime;
    isSolved = (status.problem==0);
    if ~isSolved
        message = ['[Human CEMPC solver issue] ', status.info ];
        warning(message);
    end
    
    if verbose
        disp(['Human CEMPC modeling time: ', num2str(total_time-stime),...
            ' s'])
    	disp(['Human CEMPC solving time: ', num2str(stime), ' s'])
    end

    %----------------------------------------------------------------------
    % Return values
    %----------------------------------------------------------------------
    xH_Sol = value(xH);
    uH_Sol = value(uH);
    VSol = value(objective);
end
