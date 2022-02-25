function u_ik = getiLQcontrol(x_k, x_ref_k, u_ref_k, P_ik, alpha_ik,...
    alpha_scaling)
%	Computes iLQ control law.
%
% Author: Haimin Hu (last modified 2021.11.11)
     u_ik = u_ref_k - P_ik * (x_k - x_ref_k) - alpha_scaling * alpha_ik;
end

