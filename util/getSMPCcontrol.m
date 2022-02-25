function [uR_Sol, VSol, stime, stree, flag] = getSMPCcontrol(x0, xF_R,...
    theta_distr_0, M_distr_0, Nd, Ne, QR, RR, PR, Sigma_d,...
    uR_constraints, uH_constraints, ilq_solvers, planner, verbose,...
    extraArg)
%     Computes the dual SMPC policy.
%
% NOTE:
%   1. Human's controls and M samples at time t are stored in nodes whose
%   time is t+1.
%
% Author: Haimin Hu (last modified 2021.12.13)

    yalmip('clear')
    
    params = planner.params;
    
    if strcmp(extraArg.opt_mode, 'std')
        stree_ws = extraArg.stree;
    end
    
    flag = 0;
    
    %----------------------------------------------------------------------
    % Solver setup
    %----------------------------------------------------------------------
    % Soft constraint weights.
    w_uH   = extraArg.w_uH;     % human's control
    w_RB_l = extraArg.w_RB_l;	% road boundary (linear penalty)
    w_RB_q = extraArg.w_RB_q;	% road boundary (quadratic penalty)
    w_CA_l = extraArg.w_CA_l;	% collision avoidance (linear penalty)
    w_CA_q = extraArg.w_CA_q;	% collision avoidance (quadratic penalty)
    
    % Define solver settings.
    options = sdpsettings('verbose', 0, 'solver', 'snopt', 'usex0', 1,...
        'cachesolvers', 1, 'showprogress', 0, 'debug', 0);
    
    %----------------------------------------------------------------------
    % iLQ solutions
    %----------------------------------------------------------------------
    % Systems.
    if M_distr_0(1) >= M_distr_0(2)
        ilq_solver_AH_tmp = ilq_solvers.AH_l; % Altruistic Human, M = 'l'
    else
        ilq_solver_AH_tmp = ilq_solvers.AH_r; % Altruistic Human, M = 'r'
    end
    
    jointSys = ilq_solver_AH_tmp.dynamics;
    ts = jointSys.ts;
    
    H_id = planner.H_id;
    subsys_H = ilq_solver_AH_tmp.dynamics.subsystems{H_id};
    xH_dims = jointSys.x_dims{H_id};
    uH_dims = jointSys.u_dims{H_id};
    
    R_id = planner.R_id;
    subsys_R = ilq_solver_AH_tmp.dynamics.subsystems{R_id};
    xR_dims = jointSys.x_dims{R_id};
    uR_dims = jointSys.u_dims{R_id};
    
    % Nominal states (for warm-start).
    xH_ref_AH = ilq_solver_AH_tmp.best_operating_point.xs(xH_dims,:);
%     xR_ref_AH = ilq_solver_AH_tmp.best_operating_point.xs(xR_dims,:);
    
    % Nominal inputs (for warm-start).
    uH_ref_AH = ilq_solver_AH_tmp.best_operating_point.us(uH_dims,:);
%     uR_ref_AH = ilq_solver_AH_tmp.best_operating_point.us(uR_dims,:);
    
    %----------------------------------------------------------------------
    % Construct a scenario tree
    %----------------------------------------------------------------------
    % Define the root node.
    path_prob_root = 1;
    theta_dim = 2;
    M_dim = 2;
    root_node = node(1, 0, 1, path_prob_root, [], [],...
        planner.xR_dim, planner.xH_dim, planner.uR_dim, planner.uH_dim,...
        theta_dim, M_dim, planner.xR_dims, planner.xH_dims,...
        planner.uR_dims, planner.uH_dims);
    
    if M_distr_0(1) >= M_distr_0(2)
        root_node.theta_distr = theta_distr_0.l;
    else
        root_node.theta_distr = theta_distr_0.r;
    end
    root_node.M_distr = M_distr_0;
    if M_distr_0(1) >= M_distr_0(2)
        root_node.M_sample = 'l';
    else
        root_node.M_sample = 'r';
    end
    
    % Initialize a scenario tree.
    stree = tree(root_node, planner);
    
    % Tree branching options (M branching/non-branching).
    if extraArg.M_branch
        % Branching M.
        M_set = ['l', 'r'];
    else
        % Non-branching M: Use the MAP value of M.
        if M_distr_0(1) >= M_distr_0(2)
            M_set = 'l';
        else
            M_set = 'r';
        end
    end
    
    % Construct the scenario tree.
	stree = stree.constructTree(Nd, Ne, M_set, extraArg, verbose);
    
    %----------------------------------------------------------------------
    % Assign probabilities of M
    %----------------------------------------------------------------------
    constraint = [];
    if extraArg.M_branch
        % Branching M.
        for i = 1:numel(stree.N)
            if stree.N(i).t == 1
                continue
            end
            
            if strcmp(extraArg.opt_mode,'ws')
                % Warmstart mode: Assign probabilities of M with the prior.
                if stree.N(i).M_sample == 'l'
                    stree.N(i).M_sample_prob = M_distr_0(1);
                elseif stree.N(i).M_sample == 'r'
                    stree.N(i).M_sample_prob = M_distr_0(2);
                end
            elseif ~extraArg.M_dual || strcmp(extraArg.opt_mode,'std')
                % Non-dual control of M: Assign probabilities of M using 
                % the warmstart results.
                ip = stree.N(i).pre_node_idx;
                if stree.N(i).M_sample == 'l'
                    stree.N(i).M_sample_prob = stree_ws.N(ip).M_distr(1);
                elseif stree.N(i).M_sample == 'r'
                    stree.N(i).M_sample_prob = stree_ws.N(ip).M_distr(2);
                end
            elseif extraArg.M_dual || strcmp(extraArg.opt_mode,'std')
                % Dual control of M: 'Optimize' M probabilities as decision
                % variables.
                ip = stree.N(i).pre_node_idx;
                if stree.N(i).M_sample == 'l'
                    stree.N(i).M_sample_prob = stree.N(ip).M_distr(1);
                elseif stree.N(i).M_sample == 'r'
                    stree.N(i).M_sample_prob = stree.N(ip).M_distr(2);
                end
            else
                % Sanity check.
                error('Invalid M mode and/or optimization mode!')
            end
        end
    else
        % Non-branching M.
        for i = 1:numel(stree.N)
            stree.N(i).M_sample_prob = 1;
        end
    end
    
    %----------------------------------------------------------------------
    % Objective function
    %----------------------------------------------------------------------
    objective = 0;
    for i = 1:numel(stree.N)
        % Terminal costs.
        if stree.N(i).t == Nd+Ne+1
            objective = objective +...
                stree.N(i).M_sample_prob * stree.N(i).path_prob *...
                (stree.N(i).xR-(xF_R(xR_dims)+[stree.N(i).xH(1);0;0;0]))'*...
                PR*(stree.N(i).xR-(xF_R(xR_dims)+[stree.N(i).xH(1);0;0;0]));
        end
        
        % Enforce human's controls.
        if stree.N(i).t > 1
            objective = objective +...
                stree.N(i).M_sample_prob * stree.N(i).path_prob *...
                w_uH * norm(stree.N(i).uH - stree.N(i).uH_nom)^2;
        end
        
        % Stage costs.
        if stree.N(i).t < Nd+Ne+1
            objective = objective +...
                stree.N(i).M_sample_prob * stree.N(i).path_prob *...
                (stree.N(i).uR' * RR * stree.N(i).uR +...
                ([stree.N(i).xH; stree.N(i).xR] - xF_R)' * QR *...
                ([stree.N(i).xH; stree.N(i).xR] - xF_R));
        end
    end
    
    %----------------------------------------------------------------------
    % Information gathering / entropy minimization (Explicit Dual Control)
    %----------------------------------------------------------------------
    if isfield(extraArg, 'weight_IG')
        for i = 1:numel(stree.N)
            if stree.N(i).t == Nd+1
                objective = objective +...
                    stree.N(i).M_sample_prob * extraArg.weight_IG *...
                    (stree.N(i).theta_distr.covar(1) +...
                     stree.N(i).theta_distr.covar(2) -...
                     (stree.N(i).M_distr(1)*log(stree.N(i).M_distr(1))+...
                      stree.N(i).M_distr(2)*log(stree.N(i).M_distr(2)))/...
                      sum(stree.N(i).M_distr));
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Inputs constraints
    %----------------------------------------------------------------------
    for i = 1:numel(stree.N)
        if stree.N(i).t < Nd+Ne+1
            % Robot's control bounds.
            constraint = [constraint uR_constraints.min<=stree.N(i).uR,...
                stree.N(i).uR<=uR_constraints.max];
        end
        
        if stree.N(i).t > 1
            % Human's rational control (linear combination of basis).
            ip = stree.N(i).pre_node_idx;
            
            iLQ_result = getiLQresults([stree.N(ip).xH; stree.N(ip).xR],...
                [], ilq_solvers,planner, stree.N(ip).t);
            
            if stree.N(i).M_sample == 'l'
                uH_AH_k = iLQ_result.AH_l.uH;
                uH_EH_k = iLQ_result.EH_l.uH;
            elseif stree.N(i).M_sample == 'r'
                uH_AH_k = iLQ_result.AH_r.uH;
                uH_EH_k = iLQ_result.EH_r.uH;
            end
            
            if strcmp(extraArg.opt_mode, 'ws')
                theta_1_k = root_node.theta_distr.mu(1);
              	theta_2_k = root_node.theta_distr.mu(2);
            elseif strcmp(extraArg.opt_mode, 'std')
                theta_1_k = stree.N(i).theta_sample(1)/...
                   (stree.N(i).theta_sample(1)+stree.N(i).theta_sample(2));
                theta_2_k = stree.N(i).theta_sample(2)/...
                   (stree.N(i).theta_sample(1)+stree.N(i).theta_sample(2));
            end

            constraint = [constraint...
                stree.N(i).uH_nom == theta_1_k*uH_AH_k+theta_2_k*uH_EH_k];

            % Human's control bounds.
            constraint = [constraint uH_constraints.min<=stree.N(i).uH,...
                stree.N(i).uH<=uH_constraints.max];
        end
    end
    
    %----------------------------------------------------------------------
    % State constraints
    %----------------------------------------------------------------------
    % Collision avoidance ellipsoids.
    scale_ellips = extraArg.scale_ellips;
    ellips = ellipsoid(zeros(2,1),...
        blkdiag(scale_ellips * params.avoid.lgt_ub,...
        scale_ellips * params.avoid.lat_bd)^2);
    [~, Q_CA] = double(ellips);
    
    for i = 1:numel(stree.N)
        if stree.N(i).t == 1
            continue
        end
        
        % Road boundaries.
        constraint = [constraint...
            params.rd_bd_min*0.9-stree.N(i).eps.RB_R<=stree.N(i).xR(2),...
            params.rd_bd_max*0.9+stree.N(i).eps.RB_R>=stree.N(i).xR(2)];
        
        constraint = [constraint...
            params.rd_bd_min*0.9-stree.N(i).eps.RB_H<=stree.N(i).xH(2),...
            params.rd_bd_max*0.9+stree.N(i).eps.RB_H>=stree.N(i).xH(2)];
       
        % Collision avoidance.
        constraint = [constraint...
            -(stree.N(i).xR(1:2)-stree.N(i).xH(1:2))' * inv(Q_CA) *...
            (stree.N(i).xR(1:2)-stree.N(i).xH(1:2))+1<=stree.N(i).eps.CA];
        
     	% Soft state constraint penalties.
        objective = objective +...
            stree.N(i).M_sample_prob * stree.N(i).path_prob *...
            (w_RB_l*stree.N(i).eps.RB_R + w_RB_q*stree.N(i).eps.RB_R^2);
        objective = objective +...
            stree.N(i).M_sample_prob * stree.N(i).path_prob *...
            (w_RB_l*stree.N(i).eps.RB_H + w_RB_q*stree.N(i).eps.RB_H^2);
        objective = objective +...
            stree.N(i).M_sample_prob * stree.N(i).path_prob *...
            (w_CA_l*stree.N(i).eps.CA + w_CA_q*stree.N(i).eps.CA^2);
    end

    %----------------------------------------------------------------------
    % Initial state constraints
    %----------------------------------------------------------------------
    for i = 1:numel(stree.N)
        if stree.N(i).t == 1
            constraint = [constraint stree.N(i).xR == x0(xR_dims)];
            constraint = [constraint stree.N(i).xH == x0(xH_dims)];
        end
    end
    
    %----------------------------------------------------------------------
    % Dynamics constraints
    %----------------------------------------------------------------------
    for i = 1:numel(stree.N)
        if stree.N(i).t > 1
            ip = stree.N(i).pre_node_idx;
            constraint = [constraint...
                stree.N(i).xR == subsys_R.integrate(stree.N(ip).xR,...
                stree.N(ip).uR, ts)];
            constraint = [constraint...
                stree.N(i).xH == subsys_H.integrate(stree.N(ip).xH,...
                stree.N(i).uH, ts)]; % Be careful with human's control.
        end
    end
    
    %----------------------------------------------------------------------
    % Belief state dynamics constraints
    %----------------------------------------------------------------------
    if strcmp(extraArg.opt_mode, 'std')
        for i = 1:numel(stree.N)
            % Dual control steps: propagate the belief state via the 
            % recursive Bayesian estimation.
            if 1 < stree.N(i).t && stree.N(i).t <= Nd+1
                % Linearize dynamics.
                ip = stree.N(i).pre_node_idx;
                [A_ip, B_cell_ip] = jointSys.linearizeDiscrete_B_cell(...
                   [stree_ws.N(ip).xH; stree_ws.N(ip).xR],...
                   [stree_ws.N(i).uH; stree_ws.N(ip).uR]);
                
                % Compute fixed basis of human's control.
                k = stree.N(ip).t;
                
                iLQ_result_ws = getiLQresults(...
                    [stree_ws.N(ip).xH; stree_ws.N(ip).xR], B_cell_ip,...
                    ilq_solvers, planner, k);
                
                uH_AH_fixed = iLQ_result_ws.AH_l.uH;
                uH_EH_fixed = iLQ_result_ws.EH_l.uH;
                
                extraArg.U_fixed = [uH_AH_fixed uH_EH_fixed];
                extraArg.mu_theta_fixed = stree_ws.N(ip).theta_distr.mu;
                
                % Propagate theta.
                iLQ_result = getiLQresults(...
                    [stree.N(ip).xH; stree.N(ip).xR], B_cell_ip,...
                    ilq_solvers, planner, k);
                
                [mu_next, covar_next] = updateTheta(...
                    stree.N(i).M_sample, stree.N(ip).theta_distr.mu,...
                    stree.N(ip).theta_distr.covar, Sigma_d,...
                    [stree.N(i).xH; stree.N(i).xR],...
                    [stree.N(i).uH; stree.N(ip).uR],...
                    A_ip * [stree.N(ip).xH; stree.N(ip).xR], B_cell_ip,...
                    iLQ_result, planner, extraArg); %!!! uH <- uH_nom

                constraint = [constraint...
                    stree.N(i).theta_distr.mu == mu_next
                    stree.N(i).theta_distr.covar == covar_next];
                
                % Propagate M.
                if extraArg.M_dual
                    extraArg.theta_distr_M_fixed =...
                        stree_ws.N(ip).theta_distr_M_ws;
                    M_distr_next = updateM(stree.N(ip).M_distr, [],...
                        [stree.N(i).xH; stree.N(i).xR], stree.N(ip).uR,...
                        A_ip * [stree.N(ip).xH; stree.N(ip).xR],...
                        B_cell_ip, iLQ_result, planner, extraArg);
                    constraint = [constraint...
                        stree.N(i).M_distr == M_distr_next];
                end
            end

            % Exploitation steps.
            if stree.N(i).t > Nd+1
                ip = stree.N(i).pre_node_idx;
                constraint = [constraint...
                  stree.N(i).theta_distr.mu==stree.N(ip).theta_distr.mu,...
                  stree.N(i).theta_distr.covar==stree.N(ip).theta_distr.covar];
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Reconstruct theta samples
    %----------------------------------------------------------------------
    if strcmp(extraArg.opt_mode, 'std')
        for i = 1:numel(stree.N)
            % Dual control steps.
            if 1 < stree.N(i).t && stree.N(i).t <= Nd+1
            	constraint = [constraint...
                   stree.N(i).theta_sample == transform_theta(stree.N(i))];
            end

            % Exploitation steps.
            if stree.N(i).t > Nd+1
            	constraint = [constraint...
                    stree.N(i).theta_sample == stree.N(i).theta_distr.mu]; 
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Warm-start
    %----------------------------------------------------------------------
    if strcmp(extraArg.opt_mode, 'ws')
        for i = 1:numel(stree.N)
            k = stree.N(i).t;
            assign(stree.N(i).xH, xH_ref_AH(:,k))
%             assign(stree.N(i).xR, xR_ref_AH(:,k))
            assign(stree.N(i).xR, extraArg.xR_ws(:,k))
            if k > 1
                assign(stree.N(i).uH, uH_ref_AH(:,k-1))
                assign(stree.N(i).uH_nom, uH_ref_AH(:,k-1))
%                 assign(stree.N(i).uR, uR_ref_AH(:,k-1))
                assign(stree.N(i).uR, extraArg.uR_ws(:,k-1))
            end
        end
    elseif strcmp(extraArg.opt_mode, 'std')
        for i = 1:numel(stree.N)
            assign(stree.N(i).xH, stree_ws.N(i).xH)
            assign(stree.N(i).xR, stree_ws.N(i).xR)
            if stree.N(i).t > 1
                assign(stree.N(i).uH, stree_ws.N(i).uH)
                assign(stree.N(i).uH_nom, stree_ws.N(i).uH_nom)
                assign(stree.N(i).uR, stree_ws.N(i).uR)
%                 assign(stree.N(i).theta_distr.mu, stree_ws.N(i).theta_distr.mu)
                assign(stree.N(i).theta_distr.covar, stree_ws.N(i).theta_distr.covar)
%                 assign(stree.N(i).theta_sample, stree_ws.N(i).theta_sample)
%                 if extraArg.M_dual
%                     assign(stree.N(i).M_distr, stree_ws.N(i).M_distr)
%                 end
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Solve the ST-SMPC problem
    %----------------------------------------------------------------------
    tic
    status = optimize(constraint, objective, options);
    total_time = toc;
    stime = status.solvertime;
    isSolved = (status.problem==0);
    if ~isSolved
        if strcmp(extraArg.opt_mode, 'ws')
            message = ['[WS-SMPC solver issue] ', status.info];
        elseif strcmp(extraArg.opt_mode, 'std')
            message = ['[Dual-SMPC solver issue] ', status.info];
        else
            message = ['[Unknown-SMPC solver issue] ', status.info];
        end
        warning(message);
        if strcmp(status.info,...
                'Maximum iterations or time limit exceeded (SNOPT)')
            flag = 1;
        end
    end

    if verbose
        if strcmp(extraArg.opt_mode, 'ws')
            disp(['WS-SMPC modeling time: ', num2str(total_time-stime),...
                ' s'])
            disp(['WS-SMPC solving time: ', num2str(stime), ' s'])
        elseif strcmp(extraArg.opt_mode, 'std')
            disp(['Dual-SMPC modeling time: ',...
                num2str(total_time-stime), ' s'])
            disp(['Dual-SMPC solving time: ', num2str(stime), ' s'])
        end
    end
    
    %----------------------------------------------------------------------
    % Retrieve value and propagate the belief states (for warm-start only)
    %----------------------------------------------------------------------
    if strcmp(extraArg.opt_mode, 'ws')
        for i = 1:numel(stree.N)
            stree.N(i).xH = value(stree.N(i).xH);
            stree.N(i).xR = value(stree.N(i).xR);
            stree.N(i).uH = value(stree.N(i).uH);
            stree.N(i).uH_nom = value(stree.N(i).uH_nom);
            stree.N(i).uR = value(stree.N(i).uR);
        end
        
        stree.N(1).theta_distr_M_ws = theta_distr_0;
        stree.N(1).theta_sample = root_node.theta_distr.mu;
        for i = 1:numel(stree.N)
            if stree.N(i).t > 1
                ip = stree.N(i).pre_node_idx;
                
                % Get iLQ results.
                [A_ip, B_cell_ip] = jointSys.linearizeDiscrete_B_cell(...
                    [stree.N(ip).xH; stree.N(ip).xR],...
                    [stree.N(i).uH; stree.N(ip).uR]);
                
                iLQ_result = getiLQresults(...
                        [stree.N(ip).xH; stree.N(ip).xR], B_cell_ip,...
                        ilq_solvers, planner, stree.N(i).t);
                    
                % Propagate theta.
                if extraArg.M_branch
                    for M = M_set
                        if M == 'l'
                            theta_distr_now=stree.N(ip).theta_distr_M_ws.l;
                        elseif M == 'r'
                            theta_distr_now=stree.N(ip).theta_distr_M_ws.r;
                        end

                        [mu_next, covar_next] = updateTheta(...
                            M, theta_distr_now.mu,...
                            theta_distr_now.covar, Sigma_d,...
                            [stree.N(i).xH; stree.N(i).xR],...
                            [stree.N(i).uH; stree.N(ip).uR],...
                            A_ip*[stree.N(ip).xH; stree.N(ip).xR],...
                            B_cell_ip, iLQ_result, planner, []);
                        
                        if M == 'l'
                            stree.N(i).theta_distr_M_ws.l.mu = mu_next;
                            stree.N(i).theta_distr_M_ws.l.covar =...
                                covar_next;
                        elseif M == 'r'
                            stree.N(i).theta_distr_M_ws.r.mu = mu_next;
                            stree.N(i).theta_distr_M_ws.r.covar =...
                                covar_next;
                        end
                        
                        if stree.N(i).M_sample == M
                            theta_distr_i.mu = mu_next;
                            theta_distr_i.covar = covar_next;
                            stree.N(i).theta_distr = theta_distr_i;
                            stree.N(i).theta_sample =...
                                transform_theta(stree.N(i));
                        end
                    end
                else
                    [mu_next, covar_next] = updateTheta(...
                        stree.N(i).M_sample, stree.N(ip).theta_distr.mu,...
                        stree.N(ip).theta_distr.covar, Sigma_d,...
                        [stree.N(i).xH; stree.N(i).xR],...
                        [stree.N(i).uH; stree.N(ip).uR],...
                        A_ip*[stree.N(ip).xH; stree.N(ip).xR],...
                        B_cell_ip, iLQ_result, planner, []);

                    theta_distr_i.mu = mu_next;
                    theta_distr_i.covar = covar_next;
                    stree.N(i).theta_distr = theta_distr_i;
                    stree.N(i).theta_sample = transform_theta(stree.N(i));
                end
                
                % Propagate M.
                if extraArg.M_branch
                    M_distr_next = updateM(stree.N(ip).M_distr,...
                        stree.N(ip).theta_distr_M_ws,...
                        [stree.N(i).xH; stree.N(i).xR], stree.N(ip).uR,...
                        A_ip*[stree.N(ip).xH; stree.N(ip).xR],...
                        B_cell_ip, iLQ_result, planner, []);
                    
                    stree.N(i).M_distr = M_distr_next;
                end
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Return values
    %----------------------------------------------------------------------
    uR_Sol = value(stree.N(1).uR);
    VSol = value(objective);
    
    function theta_out = transform_theta(node)
        theta_out = node.theta_distr.mu + sqrtm(node.theta_distr.covar)*...
            node.theta_sample_sn;
    end
end

