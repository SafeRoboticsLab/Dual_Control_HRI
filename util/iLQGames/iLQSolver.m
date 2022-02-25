classdef iLQSolver
    % Iterative LQ solver.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-10, Last modified: 2021-11-10
    
    properties
        dynamics                % Joint dynamics (an ProductMultiPlayerDynamicalSystem object)
        x0                      % Initial states
        Ps_cell                 % Cell of P matrices (feedback gains, row = player, col = horizon)
        alphas_cell             % Cell of alphas (feedforward terms, row = player, col = horizon)
        Zs_cell                 % Cell of Zs (value function matrices, row = player, col = horizon)
        player_costs_cell       % Cell of PlayerCost (row = player, col = horizon)
        u_constraints_cell      % Cell of constraints on controls (row = player)
        horizon                 % Game horizon
        num_players             % Number of players
        alpha_scaling           % Step size on the alpha
        max_iteration           % Maximum number of iterations
        last_operating_point    % Previous operating points
        current_operating_point % Current operating points
        best_operating_point    % Best operating points
        tolerence_percentage    % Convergence tolerence percentage ([0,1])
        name                    % Name of the object
    end
    
    methods
        function obj = iLQSolver(dynamics, x0, Ps_cell, alphas_cell,...
                player_costs_cell, u_constraints_cell, alpha_scaling,...
                max_iteration, tolerence_percentage, name)
            % Construct an instance of this class.
            obj.dynamics = dynamics;
            obj.x0 = x0;
            obj.Ps_cell = Ps_cell;
            obj.alphas_cell = alphas_cell;
            obj.player_costs_cell = player_costs_cell;
            obj.u_constraints_cell = u_constraints_cell;
            obj.num_players = size(player_costs_cell, 1);
            obj.horizon = size(player_costs_cell, 2);
            obj.alpha_scaling = alpha_scaling;
            obj.max_iteration = max_iteration;
            obj.tolerence_percentage = tolerence_percentage;
            obj.name = name;
            
            % Operating points consist of x, u, costs, Z.
            obj.last_operating_point = [];
            obj.current_operating_point = [];
            obj.best_operating_point = [];
        end
        
        function obj = run(obj, verbose)
            % Run the algorithm for the specified parameters.
            iteration = 0;
            Ps = obj.Ps_cell;
            alphas = obj.alphas_cell;
            Zs = cell(obj.num_players, obj.horizon + 1);
            while ~(obj.is_converged_cost(verbose)) &&...
                    (iteration<=obj.max_iteration)

                % (1) Compute current operating point and update last one.
                [xs, us, costs] = obj.compute_operating_point();
                obj.last_operating_point = obj.current_operating_point;
                obj.current_operating_point.xs = xs;
                obj.current_operating_point.us = us;
                obj.current_operating_point.costs = costs;
                obj.current_operating_point.Ps = Ps;
                obj.current_operating_point.alphas = alphas;
                obj.current_operating_point.Zs = Zs;
                
                % (2) Linearize about this operating point. Make sure to
                % stack appropriately since we will concatenate state 
                % vectors but not control vectors, so that
                %    ``` x_{k+1} - xs_k = A_k (x_k - xs_k) +
                %          sum_i Bi_k (ui_k - uis_k) ```
                As = cell(1, obj.horizon);
                Bs = cell(obj.num_players, obj.horizon);
                for k = 1:obj.horizon
                    [A, B] = obj.dynamics.linearizeDiscrete_B_cell(...
                            xs(:,k), us(:,k));
                    for i = 1:obj.num_players
                        As{k} = A;
                        Bs{i,k} = B{i};
                    end
                end
                
                % (3) Quadraticize costs.
                ls = cell(obj.num_players, obj.horizon);
                Qs = cell(obj.num_players, obj.horizon);
                Rs = cell(obj.num_players, obj.horizon);
                for i = 1:obj.num_players
                    for k = 1:obj.horizon
                        us_ik = us(obj.dynamics.u_dims{i}, k);
                        [~, l, Q, R] =...
                   obj.player_costs_cell{i,k}.quadraticize(xs(:,k), us_ik);
                        ls{i,k} = l;
                        Qs{i,k} = Q;
                        Rs{i,k} = R;
                    end
                end
                
                % (4) Compute feedback Nash equilibrium of the resulting 
                % LQ game.
                [Ps, alphas, Zs] = obj.solve_lq_game(As, Bs, ls, Qs, Rs);
                
                % Report time-accumulated cost for all players.
                if verbose
                    disp(['Total cost for all players: ',...
                        num2str(sum(costs,2)')])
                end
                
                % Update the member variables.
                obj.Ps_cell = Ps;
                obj.alphas_cell = alphas;
                
                % (5) Check if the current operating point is the best.
                if isempty(obj.best_operating_point)
                    obj.best_operating_point = obj.current_operating_point;
                    best_cost = sum(costs(:));
                elseif isempty(obj.best_operating_point.Zs{1,1})
                    obj.best_operating_point = obj.current_operating_point;
                    best_cost = sum(costs(:));
                elseif sum(costs(:)) < best_cost &&...
                       ~isempty(obj.current_operating_point.Zs{1,1})
                    obj.best_operating_point = obj.current_operating_point;
                    best_cost = sum(costs(:));
                end

                iteration = iteration + 1;
            end
        end
        
        function [xs, us, costs] = compute_operating_point(obj)
            % Compute current operating point by propagating through
            % the dynamics.
            % OUTPUT:
            %   xs: x_dim-by-horizon
            %   us: u_dim-by-horizon
            %   costs: num_player-by-horizon
            xs = obj.x0;
            us = [];
            costs = [];
            
            for k = 1:obj.horizon
                % Get current state and control.
                if isempty(obj.current_operating_point)
                    current_x = zeros(obj.dynamics.x_dim,1);
                    current_u = zeros(obj.dynamics.u_dim,1);
                else
                    current_x = obj.current_operating_point.xs(:,k);
                    current_u = obj.current_operating_point.us(:,k);
                end
                
                % Compute control for each player.
                uk = [];
                for i = 1:obj.num_players
                    % Feedback control law for Player i.
                    x_ref = current_x;
                    u_ref = current_u(obj.dynamics.u_dims{i});
                    P_ik = obj.Ps_cell{i,k};
                    alpha_ik = obj.alphas_cell{i,k};
                    ui = getiLQcontrol(xs(:,k), x_ref, u_ref, P_ik,...
                        alpha_ik, obj.alpha_scaling);
                    
                    % Clip the control.
                    if ~isempty(obj.u_constraints_cell{i})
                        ui = max(min(ui, obj.u_constraints_cell{i}.max),...
                            obj.u_constraints_cell{i}.min);
                    end
                    
                    uk = [uk; ui];
                end
                us = [us uk];
                
                % Compute cost for each player.
                costk = [];
                for i = 1:obj.num_players
                    costi = obj.player_costs_cell{i,k}.cost(xs(:,k),...
                        uk(obj.dynamics.u_dims{i}));
                    costk = [costk; costi];
                end
                costs = [costs costk];
                
                % Evolve the dynamics.
                xs = [xs obj.dynamics.integrate(xs(:,k), uk,...
                	obj.dynamics.ts)];
            end
        end
        
        function flag = is_converged_cost(obj, verbose)
            % Check if the last two operating points are close enough 
            % using costs.
            if isempty(obj.last_operating_point)
                flag = false;
            else
                last_costs = obj.last_operating_point.costs;
                current_costs = obj.current_operating_point.costs;
                for i = 1:obj.num_players
                    last_cost = sum(last_costs(i,:));
                    current_cost = sum(current_costs(i,:));
                    if norm((current_cost - last_cost)/last_cost) >...
                            obj.tolerence_percentage
                        if verbose
                            disp(['Player ', num2str(i),...
                              ': (current_cost-last_cost)/last_cost = ',...
                          num2str(((current_cost-last_cost)/last_cost)),...
                          '. Convergence criterion not met!'])
                        end
                        flag = false;
                        return
                    end
                end
                flag = true;
            end
        end
        
        function [Ps, alphas, Zs] = solve_lq_game(obj, As, Bs, ls, Qs, Rs)
            % Function to solve time-varying finite horizon LQ game.
            % Please refer to Corollary 6.1 on pp. 279 of Dynamic 
            % Noncooperative Game Theory (second edition) by Tamer Basar 
            % and Geert Jan Olsder.
            
            % Initialization.
            Ps = cell(obj.num_players, obj.horizon);
            alphas = cell(obj.num_players, obj.horizon);
            Zs = cell(obj.num_players, obj.horizon + 1);
            zetas = cell(obj.num_players, obj.horizon + 1);
            for i = 1:obj.num_players
                Zs{i,end} = Qs{i,end};
                zetas{i,end} = ls{i,end};
            end
            
            % Cache dimensions.
            u_dims = obj.dynamics.u_dims;
            
            % Note: notation and variable naming closely follows that 
            % introduced in the "Preliminary Notation for Corollary 6.1" 
            % section, which may be found on pp. 279 of Basar and Olsder.
            % NOTE: we will assume that `c` from Basar and Olsder is 
            % always `0`.
            
            % Backward coupled-Riccati
            for k = obj.horizon:-1:1
                
                % Compute Ps given previously computed Zs.
                % Refer to equation 6.17a in Basar and Olsder.
                % This will involve solving a system of matrix linear 
                % equations of the form:
                % [S1s; S2s; ...] * [P1; P2; ...] = [Y1; Y2; ...].
                S = [];
                for i = 1:obj.num_players
                    Sis = [];
                    for j = 1:obj.num_players
                        if j == 1
                            Sis = [Sis Rs{i,k}+Bs{i,k}'*Zs{i,k+1}*Bs{i,k}];
                        else
                            Sis = [Sis Bs{i,k}'*Zs{i,k+1}*Bs{j,k}];
                        end
                    end
                    S = [S; Sis];
                end
                
                Y = [];
                for i = 1:obj.num_players
                    Y = [Y; Bs{i,k}'*Zs{i,k+1}*As{k}];
                end
                
%                 P = S \ Y;
                P = pinv(S)*Y;
                
                for i = 1:obj.num_players
                    Ps{i,k} = P(u_dims{i},:); 
                end
                
                % Compute F_k = A_k - B1_k P1_k - B2_k P2_k.
                % This is eq. 6.17c from Basar and Olsder.
                F = As{k};
                for i = 1:obj.num_players
                    F = F - Bs{i,k}*Ps{i,k};
                end
                
                % Compute Zs to be the next step earlier in time.
                for i = 1:obj.num_players
                    Z = F'*Zs{i,k+1}*F+Qs{i,k}+Ps{i,k}'*Rs{i,k}*Ps{i,k};
                    Zs{i,k} = Z;
                end

                % Compute alphas using previously computed zetas.
                % Refer to equation 6.17d in Basar and Olsder.
                % This will involve solving a system of linear matrix
                % equations of the form:
                % [S1s; S2s; ...] * [alpha1; alpha2; ..] = [Y1; Y2; ...].
                % In fact, this is the same S matrix as before (just a 
                % different Y).
                Y = [];
                for i = 1:obj.num_players
                    Y = [Y; Bs{i,k}'*zetas{i,k+1}];
                end
                
%                 alpha = S \ Y;
                alpha = pinv(S)*Y;
                
                for i = 1:obj.num_players
                    alphas{i,k} = alpha(u_dims{i},:); 
                end
                
                % Compute beta_k = -B1_k alpha1 - B2_k alpha2_k.
                % This is eq. 6.17f in Basar and Olsder (with `c = 0`).
                beta = 0;
                for i = 1:obj.num_players
                    beta = beta - Bs{i,k}*alphas{i,k};
                end
                
                % Compute zetas to be the next step earlier in time.
                % This is Remark 6.3 in Basar and Olsder.
                for i = 1:obj.num_players
                    zeta = F'*(zetas{i,k+1}+Zs{i,k+1}*beta)+ls{i,k} +...
                        Ps{i,k}'*Rs{i,k}*alphas{i,k};
                    zetas{i,k} = zeta;
                end
            end
        end
    end % end methods
end % end class

