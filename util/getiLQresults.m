function iLQ_result = getiLQresults(x, B_cell, ilq_solvers, planner, t)
%	Get iLQ computation results.
    
    ilq_solver_AH_l = ilq_solvers.AH_l;
    ilq_solver_EH_l = ilq_solvers.EH_l;
    ilq_solver_AH_r = ilq_solvers.AH_r;
    ilq_solver_EH_r = ilq_solvers.EH_r;
    
    xH_dims = planner.xH_dims;
    uH_dims = planner.uH_dims;
    
    H_id = planner.H_id;
    R_id = planner.R_id;
    
    if H_id ~= 1 || R_id ~= 2
        error('Incorrect subsystem index!') % sanity check
    end
    
    if ~isempty(B_cell)
        BH = B_cell{H_id};
    end
    
    % --------- M = 'l' ---------
    % Altruistic human, M = 'l'.
    x_nom_AH_l = ilq_solver_AH_l.best_operating_point.xs(:, t);
    u_nom_AH_l = ilq_solver_AH_l.best_operating_point.us(:, t);
    P_H_AH_l = [ilq_solver_AH_l.best_operating_point.Ps{1, t};
                ilq_solver_AH_l.best_operating_point.Ps{2, t}];
    alpha_H_AH_l = [ilq_solver_AH_l.best_operating_point.alphas{1, t};...
                    ilq_solver_AH_l.best_operating_point.alphas{2, t}];
    Z_H_AH_l = ilq_solver_AH_l.best_operating_point.Zs{H_id, t};
    
    uH_AH_l = getiLQcontrol(x, x_nom_AH_l, u_nom_AH_l, P_H_AH_l,...
        alpha_H_AH_l, planner.alpha_scaling);
    uH_AH_l = uH_AH_l(uH_dims);
    
    % Egoistic Human, M = 'l'.
    x_nom_EH_l = ilq_solver_EH_l.best_operating_point.xs(:, t);
    u_nom_EH_l = ilq_solver_EH_l.best_operating_point.us(:, t);
    P_H_EH_l   = ilq_solver_EH_l.best_operating_point.Ps{1, t};
    alpha_H_EH_l = ilq_solver_EH_l.best_operating_point.alphas{1, t};
    Z_H_EH_l = ilq_solver_EH_l.best_operating_point.Zs{1, t};
    
    uH_EH_l = getiLQcontrol(x(xH_dims), x_nom_EH_l, u_nom_EH_l,...
        P_H_EH_l, alpha_H_EH_l, planner.alpha_scaling);
    
    % --------- M = 'r' ---------
    % Altruistic human, M = 'r'.
    x_nom_AH_r = ilq_solver_AH_r.best_operating_point.xs(:, t);
    u_nom_AH_r = ilq_solver_AH_r.best_operating_point.us(:, t);
    P_H_AH_r = [ilq_solver_AH_r.best_operating_point.Ps{1, t};
                ilq_solver_AH_r.best_operating_point.Ps{2, t}];
    alpha_H_AH_r = [ilq_solver_AH_r.best_operating_point.alphas{1, t};...
                    ilq_solver_AH_r.best_operating_point.alphas{2, t}];
    Z_H_AH_r = ilq_solver_AH_r.best_operating_point.Zs{H_id, t};
    
    uH_AH_r = getiLQcontrol(x, x_nom_AH_r, u_nom_AH_r, P_H_AH_r,...
        alpha_H_AH_r, planner.alpha_scaling);
    uH_AH_r = uH_AH_r(uH_dims);
    
    % Egoistic Human, M = 'l'.
    x_nom_EH_r = ilq_solver_EH_r.best_operating_point.xs(:, t);
    u_nom_EH_r = ilq_solver_EH_r.best_operating_point.us(:, t);
    P_H_EH_r   = ilq_solver_EH_r.best_operating_point.Ps{1, t};
    alpha_H_EH_r = ilq_solver_EH_r.best_operating_point.alphas{1, t};
    Z_H_EH_r = ilq_solver_EH_r.best_operating_point.Zs{1, t};
    
    uH_EH_r = getiLQcontrol(x(xH_dims), x_nom_EH_r, u_nom_EH_r,...
        P_H_EH_r, alpha_H_EH_r, planner.alpha_scaling);
    
    % Return results.
    iLQ_result.AH_l.uH = uH_AH_l;
    iLQ_result.EH_l.uH = uH_EH_l;
    iLQ_result.AH_r.uH = uH_AH_r;
    iLQ_result.EH_r.uH = uH_EH_r;
    
    if ~isempty(B_cell)
        Sigma_AH_l = planner.RH + BH'*Z_H_AH_l*BH;
        iLQ_result.AH_l.Sigma = Sigma_AH_l;

        Sigma_EH_l = planner.RH + BH(1:4,:)'*Z_H_EH_l*BH(1:4,:);
        iLQ_result.EH_l.Sigma = Sigma_EH_l;
        
        Sigma_AH_r = planner.RH + BH'*Z_H_AH_r*BH;
        iLQ_result.AH_r.Sigma = Sigma_AH_r;

        Sigma_EH_r = planner.RH + BH(1:4,:)'*Z_H_EH_r*BH(1:4,:);
        iLQ_result.EH_r.Sigma = Sigma_EH_r;
    end
end

