classdef SemiquadraticCost < Cost
    % Semiquadratic cost, derived from Cost base class. Implements a
	% cost function that is flat below a threshold and quadratic above, in 
    % the given dimension.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        dimension       % Dimension to add cost
        threshold       % Value above which to impose quadratic cost
        oriented_right  % Boolean flag determining which side of threshold
                        % to penalize
        type            % 'state' or 'control'
    end
    
    methods
        function obj = SemiquadraticCost(dimension, threshold,...
                oriented_right, type, name, x_dim, u_dim)
            % Constructor.
            %   Initialize with dimension to add cost to and threshold 
            %   above which to impose quadratic cost.
            super_args{1} = name;
            super_args{2} = x_dim;
            super_args{3} = u_dim;
            obj@Cost(super_args{:});
            
            obj.dimension = dimension;
            obj.threshold = threshold;
            obj.oriented_right = oriented_right;
            obj.type = type;
            
            % Define symbolic gradients and Hessians.
            x = sym('x', [obj.x_dim 1]);
            u = sym('u', [obj.u_dim 1]);
            if strcmp(obj.type,'state')
                if obj.oriented_right
                    l = symfun(piecewise(x(obj.dimension)>obj.threshold,...
                        (x(obj.dimension)-obj.threshold)^2, 0), x);
                else
                    l = symfun(piecewise(x(obj.dimension)<obj.threshold,...
                        (x(obj.dimension)-obj.threshold)^2, 0), x);
                end
            elseif strcmp(obj.type,'control')
                if obj.oriented_right
                    l = symfun(piecewise(u(obj.dimension)>obj.threshold,...
                        (u(obj.dimension)-obj.threshold)^2, 0), u);
                else
                    l = symfun(piecewise(u(obj.dimension)<obj.threshold,...
                        (u(obj.dimension)-obj.threshold)^2, 0), u);
                end
            else
                error('Invalid type')
            end
            
            file_name = "./util/iLQGames/Costs/SemiquadraticCost_" + name;
            
            obj.dldx = matlabFunction(gradient(l,x),'Vars',{x},...
                'file', file_name + "_dldx.m");
            obj.Hx   = matlabFunction(hessian(l,x),'Vars',{x},...
                'file', file_name + "_Hx.m");
            obj.Hu   = matlabFunction(hessian(l,u),'Vars',{u,x},...
                'file', file_name + "_Hu.m");
        end
        
        function out = cost(obj, xu)
            % Evaluate this cost function on the given input and time, 
            % which might either be a state `x` or a control `u`. Hence 
            % the input is named `xu`.
            if obj.oriented_right
                if xu(obj.dimension) > obj.threshold
                    out = (xu(obj.dimension) - obj.threshold)^2;
                else
                    out = 0;
                end
            else
                if xu(obj.dimension) < obj.threshold
                    out = (xu(obj.dimension) - obj.threshold)^2;
                else
                    out = 0;
                end
            end
        end
        
        function out = get_Hu(obj, u0)
            % Evaluate hessian Hu at u0.
            out = obj.Hu(u0, zeros(obj.x_dim,1));
        end
    end % end methods
end % end class

