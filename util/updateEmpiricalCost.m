function J_em = updateEmpiricalCost(J_em, x, xF_R, uR, QR, RR)
%     Update the robot's empirical (closed-loop) cost.
%
% Author: Haimin Hu (last modified 2021.11.11)

    J_em = J_em + (x - xF_R)' * QR * (x - xF_R) + uR' * RR * uR;
end

