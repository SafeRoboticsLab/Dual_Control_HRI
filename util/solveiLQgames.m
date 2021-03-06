function ilq_solvers = solveiLQgames(x, xH_dims, jointSys, jointSys_EH,...
    Ps_cell, Ps_cell_EH, alphas_cell, alphas_cell_EH,...
    player_costs_cell_l, player_costs_cell_r, player_costs_cell_EH_l,...
    player_costs_cell_EH_r, u_constraints_cell, alpha_scaling,...
    max_iteration, tolerence_percentage, verbose)
% Solve iLQGames for different theta's and M's.
%
% Author: Haimin Hu (last modified 2021.11.11)

    % -> Human targets the left lane (M = 'l').
    %   - Altruistic Human.
    ilq_solver_AH_l = iLQSolver(jointSys, x, Ps_cell, alphas_cell,...
        player_costs_cell_l, u_constraints_cell, alpha_scaling,...
        max_iteration, tolerence_percentage, 'ilq_solver_AH_l');
    ilq_solver_AH_l = ilq_solver_AH_l.run(verbose);

    %   - Egoistic Human.
    ilq_solver_EH_l = iLQSolver(jointSys_EH, x(xH_dims), Ps_cell_EH,...
        alphas_cell_EH, player_costs_cell_EH_l, u_constraints_cell,...
        alpha_scaling, max_iteration, tolerence_percentage,...
        'ilq_solver_EH_l');
    ilq_solver_EH_l = ilq_solver_EH_l.run(verbose);
    
    % -> Human targets the right lane (M = 'r').
    %   - Altruistic Human.
    ilq_solver_AH_r = iLQSolver(jointSys, x, Ps_cell, alphas_cell,...
        player_costs_cell_r, u_constraints_cell, alpha_scaling,...
        max_iteration, tolerence_percentage, 'ilq_solver_AH_r');
    ilq_solver_AH_r = ilq_solver_AH_r.run(verbose);

    %   - Egoistic Human.
    ilq_solver_EH_r = iLQSolver(jointSys_EH, x(xH_dims), Ps_cell_EH,...
        alphas_cell_EH, player_costs_cell_EH_r, u_constraints_cell,...
        alpha_scaling, max_iteration, tolerence_percentage,...
        'ilq_solver_EH_r');
    ilq_solver_EH_r = ilq_solver_EH_r.run(verbose);
    
    ilq_solvers.AH_l = ilq_solver_AH_l;
    ilq_solvers.AH_r = ilq_solver_AH_r;
    ilq_solvers.EH_l = ilq_solver_EH_l;
    ilq_solvers.EH_r = ilq_solver_EH_r;
end

