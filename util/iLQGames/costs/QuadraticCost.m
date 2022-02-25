classdef QuadraticCost < Cost
    % Quadratic cost, derived from Cost base class.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        dimensions      % dimensions to add cost
        origin          % Value along the specified dim where the cost is 0
        type            % 'state' or 'control'
        P               % Cost matrix
    end
    
    methods
        function obj = QuadraticCost(dimensions, origin, type, P,...
                name, x_dim, u_dim)
            % Constructor.
            %   Initialize with dimensions to add cost to and origin to 
            %   center the quadratic cost about.
            super_args{1} = name;
            super_args{2} = x_dim;
            super_args{3} = u_dim;
            obj@Cost(super_args{:});
            
            obj.dimensions = dimensions;
            obj.origin = origin;
            obj.type = type;
            obj.P = P;
            
            % Define symbolic gradients and Hessians.
            x = sym('x', [obj.x_dim 1]);
            u = sym('u', [obj.u_dim 1]);
            if strcmp(obj.type,'state')
                l = (x(obj.dimensions) - obj.origin)'*obj.P*...
                    (x(obj.dimensions) - obj.origin);
            elseif strcmp(obj.type,'control')
                l = (u(obj.dimensions) - obj.origin)'*obj.P*...
                    (u(obj.dimensions) - obj.origin);
            else
                error('Invalid type')
            end
            obj.dldx = matlabFunction(gradient(l,x),'Vars',{x});
            obj.Hx   = matlabFunction(hessian(l,x),'Vars',{x});
            obj.Hu   = matlabFunction(hessian(l,u),'Vars',{u});
        end
        
        function out = cost(obj, xu)
            % Evaluate this cost function on the given input and time, 
            % which might either be a state `x` or a control `u`. Hence 
            % the input is named `xu`.
            out = (xu(obj.dimensions) - obj.origin)'*obj.P*...
                  (xu(obj.dimensions) - obj.origin);
        end
    end % end methods
end % end class

