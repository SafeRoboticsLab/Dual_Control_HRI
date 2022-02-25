function M_distr = updateM(M_distr_now, theta_distr_now, x_next, uR,...
    f_x, B_cell, iLQ_result, planner, extraArg)
%   Update the distribution of M.
%
% Author: Haimin Hu (last modified 2021.11.11)
  
    fixed_values = 0;
    if isfield(extraArg, 'fixed_values')
        if extraArg.fixed_values
            fixed_values = 1;
            theta_distr_M_fixed = extraArg.theta_distr_M_fixed;
        end
    end

    xH_dims = planner.xH_dims;

    H_id = planner.H_id;
    R_id = planner.R_id;
    
    BH = B_cell{H_id};
    BR = B_cell{R_id};
    
    if fixed_values
        mu_theta_l = theta_distr_M_fixed.l.mu;
        mu_theta_r = theta_distr_M_fixed.r.mu;
    else
        mu_theta_l = theta_distr_now.l.mu;
        mu_theta_r = theta_distr_now.r.mu;
    end
    
    % Get iLQ results.
    uH_AH_l = iLQ_result.AH_l.uH;
    uH_EH_l = iLQ_result.EH_l.uH;
    uH_l = mu_theta_l(1) * uH_AH_l + mu_theta_l(2) * uH_EH_l;
    
    uH_AH_r = iLQ_result.AH_r.uH;
    uH_EH_r = iLQ_result.EH_r.uH;
    uH_r = mu_theta_r(1) * uH_AH_r + mu_theta_r(2) * uH_EH_r;
    
    % Propagate states with different M's.
    x_next_l = f_x + BH * uH_l + BR * uR;
    x_next_r = f_x + BH * uH_r + BR * uR;
    
    % Distribution update.
    if isfield(extraArg, 'scale')
        scale = extraArg.scale;
    else
        scale = 1e1;
    end
    
    % Compute surrogate weights.
%     weight_l = exp(-scale*norm(x_next_l(xH_dims) - x_next(xH_dims))^2);
%     weight_r = exp(-scale*norm(x_next_r(xH_dims) - x_next(xH_dims))^2);
    weight_l = exp(-scale*norm(x_next_l(2:3) - x_next(2:3))^2);
    weight_r = exp(-scale*norm(x_next_r(2:3) - x_next(2:3))^2);
    denom = weight_l * M_distr_now(1) + weight_r * M_distr_now(2);
    
    M_distr = [weight_l * M_distr_now(1) / denom;
               weight_r * M_distr_now(2) / denom];
           
    M_distr = (1-planner.M_eps)*M_distr + planner.M_eps*...
        planner.M_distr_const;
end

