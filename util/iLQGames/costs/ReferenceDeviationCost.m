classdef ReferenceDeviationCost < Cost
    % Reference following cost.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-09, Last modified: 2021-11-09
    
    properties
        x_ref           % Reference state (nx-by-1)
        u_ref           % Reference control (nu-by-1)
        Q               % Cost matrix of states
        R               % Cost matrix of controls
    end
    
    methods
        function obj = ReferenceDeviationCost(x_ref, u_ref, Q, R,...
                name, x_dim, u_dim)
            % Constructor
            %   NOTE: This list will be updated externally as the reference
            %         trajectory evolves with each iteration.
            super_args{1} = name;
            super_args{2} = x_dim;
            super_args{3} = u_dim;
            obj@Cost(super_args{:});
            
            obj.x_ref = x_ref;
            obj.u_ref = u_ref;
            obj.Q = Q;
            obj.R = R;
            
            % Define symbolic gradients and Hessians.
            x = sym('x', [obj.x_dim 1]);
            u = sym('u', [obj.u_dim 1]);
            l = obj.cost(x, u);
            obj.dldx = matlabFunction(gradient(l,x),'Vars',{x});
            obj.Hx   = matlabFunction(hessian(l,x),'Vars',{x});
            obj.Hu   = matlabFunction(hessian(l,u),'Vars',{u});
        end
        
        function out = cost(obj, x, u)
            % Evaluate this cost function on the given vector and time.
            out = (x-obj.x_ref)'*obj.Q*(x-obj.x_ref) +...
                  (u-obj.u_ref)'*obj.R*(u-obj.u_ref);
        end
    end % end methods
end % end class

