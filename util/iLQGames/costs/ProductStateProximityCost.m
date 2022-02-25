classdef ProductStateProximityCost < Cost
    % Proximity cost for state spaces that are Cartesian products of 
    % individual systems' state spaces. Penalizes
    %      ``` sum_{i \ne j} min(distance(i, j) - max_distance, 0)^2 ```
    % for all players i, j.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        px_indices      % px indices of all players
        py_indices    	% py indices of all players
        max_distance  	% Maximum value of distance to penalize
        num_players  	% Number of players
    end
    
    methods
        function obj = ProductStateProximityCost(px_indices,...
                py_indices, max_distance, name, x_dim, u_dim)
            % Constructor.
            %   Initialize with dimension to add cost to and threshold 
            %   BELOW which to impose quadratic cost.
            super_args{1} = name;
            super_args{2} = x_dim;
            super_args{3} = u_dim;
            obj@Cost(super_args{:});
            
            obj.px_indices = px_indices;
            obj.py_indices = py_indices;
            obj.max_distance = max_distance;
            obj.num_players = length(px_indices);
            
            % Define symbolic gradients and Hessians.
            x = sym('x', [obj.x_dim 1]);
            u = sym('u', [obj.u_dim 1]);
            l = 0;
            for i = 1:obj.num_players
                xi_idx = [obj.px_indices(i), obj.py_indices(i)];
                for j = 1:obj.num_players
                    if i ~= j
                        xj_idx = [obj.px_indices(j), obj.py_indices(j)];
                        
                        % Compute relative distance.
                        dx = x(xi_idx) - x(xj_idx);
                        relative_distance = norm(dx);
                        
                        % Note: min does not work with sym.
                        l_tmp = symfun(piecewise(relative_distance-...
                            obj.max_distance<=0,(relative_distance-...
                            obj.max_distance)^2,0),x);
                        
                        l = l + l_tmp;
                    end
                end
            end
            
            file_name = "./util/iLQGames/Costs/ProductStateProximityCost_" + name;
            
            obj.dldx = matlabFunction(gradient(l,x),'Vars',{x},...
                'file', file_name + "_dldx.m");
            obj.Hx   = matlabFunction(hessian(l,x),'Vars',{x},...
                'file', file_name + "_Hx.m");
            obj.Hu   = matlabFunction(hessian(l,u),'Vars',{u,x},...
                'file', file_name + "_Hu.m");
        end
        
        function total_cost = cost(obj, x)
            % Evaluate this cost function on the given state.
            %   x: concatenated state vector of all subsystems (column vector)
            total_cost = 0;
            for ii = 1:obj.num_players
                xi_idx = [obj.px_indices(ii), obj.py_indices(ii)];
                for jj = 1:obj.num_players
                    if ii ~= jj
                        xj_idx = [obj.px_indices(jj), obj.py_indices(jj)];
                        
                        % Compute relative distance.
                        dx = x(xi_idx) - x(xj_idx);
                        relative_distance = norm(dx);
                        
                        total_cost = total_cost + min(relative_distance-...
                            obj.max_distance, 0.0)^2;
                    end
                end
            end
        end
        
        function out = get_Hu(obj, u0)
            % Evaluate hessian Hu at u0.
            out = obj.Hu(u0, zeros(obj.x_dim,1));
        end
        
    end % end methods
end % end class

