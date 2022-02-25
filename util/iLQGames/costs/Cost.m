classdef Cost
    % Base class for all cost functions.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        name            % Name of the cost
        x_dim           % State dimension
        u_dim           % Control dimension
        dldx            % Gradient of the cost l w.r.t. state x
        Hx              % Hessian of the cost l w.r.t. state x
        Hu              % Hessian of the cost l w.r.t. control u
    end
    
    methods
        function obj = Cost(name, x_dim, u_dim)
            % Constructor.
            obj.name = name;
            obj.x_dim = x_dim;
            obj.u_dim = u_dim;
        end
        
        function out = get_dldx(obj, x0)
            % Evaluate gradient dldx at x0.
            out = obj.dldx(x0);
        end
        
        function out = get_Hx(obj, x0)
            % Evaluate hessian Hx at x0.
            out = obj.Hx(x0);
        end
        
        function out = get_Hu(obj, u0)
            % Evaluate hessian Hu at u0.
            out = obj.Hu(u0);
        end
    end % end methods
end % end class

