classdef PlayerCost
    % Container to hold a bunch of different Costs and keep track of the 
    % arguments to each one.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        costs           % cell of costs
        args            % cell of args `x` or `u` or `xu`
        weights         % vector of weights
    end
    
    methods
        function obj = PlayerCost()
            % Constructor.
            obj.costs = {};
            obj.args = {};
            obj.weights = [];
        end
        
        function total_cost = cost(obj, x, u)
            % Evaluate the game cost function at the current state 
            % and/or controls.
            total_cost = 0;
            for i = 1:length(obj.costs)
                current_cost = obj.costs{i};
                if strcmp(obj.args{i},'x')
                    current_term = obj.weights(i) * current_cost.cost(x);
                elseif strcmp(obj.args{i},'u')
                    current_term = obj.weights(i) * current_cost.cost(u);
                elseif strcmp(obj.args{i},'xu')
                    current_term = obj.weights(i) * current_cost.cost(x,u);
                else
                    error('Invalid args')
                end
                
                if current_term > 1e12
                    warning(["cost "+current_cost.name+" is "+...
                        num2str(current_term)])
                    warning(['Input is ', obj.args{i}])
                end
                
                total_cost = total_cost + current_term;
            end
        end
        
        function obj = add_cost(obj, cost, arg, weight)
            % Add a new cost to the game.
            obj.costs{end+1} = cost;
            obj.args{end+1} = arg;
            obj.weights = [obj.weights weight];
        end
        
        function [total_cost, dldx, Hx, Hu] = quadraticize(obj, x, u)
            % Compute a quadratic approximation to the overall cost for a
            % particular choice of operating point `x`, `u` or `xu`
            % Returns the gradient and Hessian of the overall cost s.t.:
            %
            % cost(x + dx, u + du) \approx
            %     cost(x, u) +
            %     dldx^T dx +
            %     0.5 * (dx^T Hx dx + du^T Hu du)
            
            % Evaluate cost at the operating point.
            total_cost = obj.cost(x,u);
            
            % Compute gradient and Hessians.
            dldx = zeros(length(x), 1);
            Hx = zeros(length(x), length(x));
            Hu = zeros(length(u), length(u));
            
            for i = 1:length(obj.costs)
                current_cost = obj.costs{i};
                dldx = dldx + current_cost.get_dldx(x);
                Hx = Hx + current_cost.get_Hx(x);
                Hu = Hu + current_cost.get_Hu(u);
            end
        end
    end % end methods
end % end class

