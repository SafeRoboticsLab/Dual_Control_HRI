function [uR_Sol, xR_Sol, xH_Sol, uH_Sol, VSol, status] =...
    getCEMPCcontrol(x0, xF_R, N, QR, RR, PR, uR_constraints,...
    uH_constraints, ilq_solvers, theta_distr, M_distr, planner,...
    extraArg, verbose)
%   Nominal certainty-equivalent MPC planner that accounts for human
%   responses to robot's action.
%
% Author: Haimin Hu (last modified 2021.11.16)

    yalmip('clear')
    params = planner.params;
    
    % Get ILQ solutions.
    if M_distr(1) >= M_distr(2)
        mu_theta_MAP = theta_distr.l.mu;
        ilq_solver_AH = ilq_solvers.AH_l; 
        ilq_solver_EH = ilq_solvers.EH_l;
    else
        mu_theta_MAP = theta_distr.r.mu;
        ilq_solver_AH = ilq_solvers.AH_r; 
        ilq_solver_EH = ilq_solvers.EH_r;
    end
    
    Ps_AH = ilq_solver_AH.best_operating_point.Ps;
    Ps_EH = ilq_solver_EH.best_operating_point.Ps;
    
    alphas_AH = ilq_solver_AH.best_operating_point.alphas;
    alphas_EH = ilq_solver_EH.best_operating_point.alphas;
    
    % Systems.
    jointSys = ilq_solver_AH.dynamics;
    ts = jointSys.ts;
    
    H_id = planner.H_id;
    subsys_H = ilq_solver_AH.dynamics.subsystems{H_id};
    xH_dim = subsys_H.x_dim;
    uH_dim = subsys_H.u_dim;
    xH_dims = jointSys.x_dims{H_id};
    uH_dims = jointSys.u_dims{H_id};
    
    R_id = planner.R_id;
    subsys_R = ilq_solver_AH.dynamics.subsystems{R_id};
    xR_dim = subsys_R.x_dim;
    uR_dim = subsys_R.u_dim;
    xR_dims = jointSys.x_dims{R_id};
    
    % Soft constraint weights.
    w_uH   = extraArg.w_uH;     % human's control
    w_RB_l = extraArg.w_RB_l;	% road boundary (linear penalty)
    w_RB_q = extraArg.w_RB_q;	% road boundary (quadratic penalty)
    w_CA_l = extraArg.w_CA_l;	% collision avoidance (linear penalty)
    w_CA_q = extraArg.w_CA_q;	% collision avoidance (quadratic penalty)
    
    % Define solver settings
    options = sdpsettings('verbose', 0, 'solver', 'snopt', 'usex0', 1,...
        'cachesolvers', 1);

    %----------------------------------------------------------------------
    % Define decision variables
    %----------------------------------------------------------------------
    % States.
    xR = sdpvar(xR_dim, N+1);
    xH = sdpvar(xH_dim, N+1);
    
    % Nominal states.
    xR_ref_AH = ilq_solver_AH.best_operating_point.xs(xR_dims,:);
    xH_ref_AH = ilq_solver_AH.best_operating_point.xs(xH_dims,:);
    xH_ref_EH = ilq_solver_EH.best_operating_point.xs(xH_dims,:);
    
    % Inputs.
    uR = sdpvar(uR_dim, N);
    uH = sdpvar(uH_dim, N);
    uH_nom = sdpvar(uH_dim, N);
    
    % Nominal inputs.
    uH_ref_AH = ilq_solver_AH.best_operating_point.us(uH_dims,:);
    uH_ref_EH = ilq_solver_EH.best_operating_point.us(uH_dims,:);
    
    % Soft constraint slack variables.
    eps_RB_R = sdpvar(1, N+1);
    eps_RB_H = sdpvar(1, N+1);
    eps_CA_R = sdpvar(1, N+1);

    %----------------------------------------------------------------------
    % Objective function
    %----------------------------------------------------------------------
    % Terminal cost.
    objective = (xR(:,N+1)-(xF_R(xR_dims)+[xH(1,N+1);0;0;0]))'*...
                PR * (xR(:,N+1)-(xF_R(xR_dims)+[xH(1,N+1);0;0;0]));
    
    % Tracking cost for states and controls.
    for k = 1:N
        objective = objective + uR(:,k)'*RR*uR(:,k) +...
           ([xH(:,k); xR(:,k)] - xF_R)'*QR*([xH(:,k); xR(:,k)] - xF_R);
    end
    
    % Enforce human's controls.
    for k = 1:N
        objective = objective + w_uH*norm(uH(:,k) - uH_nom(:,k))^2;
    end
    
    % Soft state constraint penalties.
    for k = 1:N+1
        objective = objective + w_RB_l*eps_RB_R(k) + w_RB_q*eps_RB_R(k)^2;
        objective = objective + w_RB_l*eps_RB_H(k) + w_RB_q*eps_RB_H(k)^2;
        objective = objective + w_CA_l*eps_CA_R(k) + w_CA_q*eps_CA_R(k)^2;
    end

    %----------------------------------------------------------------------
    % Input constraints
    %----------------------------------------------------------------------
    constraint = [];
    for k = 1:N
        % Robot's control bounds.
        constraint = [constraint uR_constraints.min <= uR(:,k),...
            uR(:,k) <= uR_constraints.max];
        
        % Human's rational control.
        uH_AH_k = getiLQcontrol(...
            [xH(:,k); xR(:,k)], [xH_ref_AH(:,k); xR_ref_AH(:,k)],...
            uH_ref_AH(:,k), Ps_AH{H_id,k}, alphas_AH{H_id,k},...
            ilq_solver_AH.alpha_scaling);
        uH_EH_k = getiLQcontrol(...
            xH(:,k), xH_ref_EH(:,k),...
            uH_ref_EH(:,k), Ps_EH{H_id,k}, alphas_EH{H_id,k},...
            ilq_solver_EH.alpha_scaling);
        
        constraint = [constraint...
            uH_nom(:,k) == mu_theta_MAP(1)*uH_AH_k + mu_theta_MAP(2)*uH_EH_k];
        
        % Human's control bounds.
        constraint = [constraint...
            uH_constraints.min <= uH(:,k), uH(:,k) <= uH_constraints.max];
    end
    
    %----------------------------------------------------------------------
    % State constraints
    %----------------------------------------------------------------------
    % Collision avoidance ellipsoids.
    scale_ellips = 1.2;
    ellips = ellipsoid(zeros(2,1),...
        blkdiag(scale_ellips*params.avoid.lgt_ub,...
                scale_ellips*params.avoid.lat_bd)^2);
    [~, Q_CA] = double(ellips);
            
    for k = 1:N+1
        % Road boundaries.
        constraint = [constraint...
            params.rd_bd_min*0.9 - eps_RB_R <= xR(2,k),...
            params.rd_bd_max*0.9 + eps_RB_R >= xR(2,k)];
        
        constraint = [constraint...
            params.rd_bd_min*0.9 - eps_RB_H <= xH(2,k),...
            params.rd_bd_max*0.9 + eps_RB_H >= xH(2,k)];
       
        % Collision avoidance.
        constraint = [constraint...
            -(xR(1:2,k)-xH(1:2,k))'*inv(Q_CA)*(xR(1:2,k)-xH(1:2,k))...
            + 1 <= eps_CA_R];
    end

    %----------------------------------------------------------------------
    % Initial state constraints
    %----------------------------------------------------------------------
    constraint = [constraint xR(:,1) == x0(xR_dims)];
    constraint = [constraint xH(:,1) == x0(xH_dims)];

    %----------------------------------------------------------------------
    % Dynamics constraints
    %----------------------------------------------------------------------
    for k = 1:N
        % Dynamics.
        constraint = [constraint...
            xR(:,k+1) == subsys_R.integrate(xR(:,k), uR(:,k), ts)];
        constraint = [constraint...
            xH(:,k+1) == subsys_H.integrate(xH(:,k), uH(:,k), ts)];
    end
    
    %----------------------------------------------------------------------
    % Warm-start
    %----------------------------------------------------------------------
%     assign(xR, ilq_solver_AH.best_operating_point.xs(xR_dims, 1:N+1))
%     assign(xH, ilq_solver_AH.best_operating_point.xs(xH_dims, 1:N+1))
%     assign(uR, ilq_solver_AH.best_operating_point.us(uR_dims, 1:N))
%     assign(uH, ilq_solver_AH.best_operating_point.us(uH_dims, 1:N))

    %----------------------------------------------------------------------
    % Solve the MPC problem
    %----------------------------------------------------------------------
    tic
    status = optimize(constraint, objective, options);
    total_time = toc;
    stime = status.solvertime;
    isSolved = (status.problem==0);
    if ~isSolved
        message = ['[CE-MPC solver issue] ', status.info ];
        warning(message);
    end
    
    if verbose
        disp(['CE-MPC modeling time: ', num2str(total_time-stime), ' s'])
    	disp(['CE-SMPC solving time: ', num2str(stime), ' s'])
    end

    %----------------------------------------------------------------------
    % Return values
    %----------------------------------------------------------------------
    xR_Sol = value(xR);
    uR_Sol = value(uR);
    xH_Sol = value(xH);
    uH_Sol = value(uH);
%     uH_nom_Sol = value(uH_nom);
    VSol = value(objective);
end