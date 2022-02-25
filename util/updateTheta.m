function [mu, Sigma] = updateTheta(M, mu_now, Sigma_now, Sigma_d,...
    x_next, u, f_x, B_cell, iLQ_result, planner, extraArg)
%	Update the conditional distribution of theta given M.
%   Assumes that Sigma_now and Sigma_d are diagonal matrices.
%
% Author: Haimin Hu (last modified 2021.11.11)
    
    fixed_values = 0;
    if isfield(extraArg, 'fixed_values')
        if extraArg.fixed_values
            fixed_values = 1;
            U_fixed = extraArg.U_fixed;
            mu_fixed = extraArg.mu_theta_fixed;
        end
    end
    
    uR_dims = planner.uR_dims;
    
    H_id = planner.H_id;
    R_id = planner.R_id;
    
    BH = B_cell{H_id};
    BR = B_cell{R_id};
    
    % Get iLQ results.
    if M == 'l'
        uH_AH = iLQ_result.AH_l.uH;
        Sigma_AH = iLQ_result.AH_l.Sigma;

        uH_EH = iLQ_result.EH_l.uH;
        Sigma_EH = iLQ_result.EH_l.Sigma;
    elseif M == 'r'
        uH_AH = iLQ_result.AH_r.uH;
        Sigma_AH = iLQ_result.AH_r.Sigma;

        uH_EH = iLQ_result.EH_r.uH;
        Sigma_EH = iLQ_result.EH_r.Sigma;
    end
    
    % Augment covariance.
    U = [uH_AH uH_EH];
    
    if fixed_values
        Sigma_d_aug = Sigma_d +...
            mu_fixed(1)^2*BH*U_fixed*Sigma_AH*(BH*U_fixed)'+...
            mu_fixed(2)^2*BH*U_fixed*Sigma_EH*(BH*U_fixed)';
    else
        Sigma_d_aug = Sigma_d+mu_now(1)^2*BH*U*Sigma_AH*(BH*U)'+...
            mu_now(2)^2*BH*U*Sigma_EH*(BH*U)';
    end
    
    Sigma_now_inv = diag(1 ./ diag(Sigma_now)');
    
    % Covariance update.
    Sigma_inv = Sigma_now_inv + (BH*U)'*inv(Sigma_d_aug)*(BH*U);
    Sigma = diag(1 ./ diag(Sigma_inv)');
    
    % Mean update.
    uR = u(uR_dims);
    mu = Sigma * ((BH*U)'*inv(Sigma_d_aug)*(x_next - f_x - BR*uR) +...
        Sigma_now_inv*mu_now);
end

