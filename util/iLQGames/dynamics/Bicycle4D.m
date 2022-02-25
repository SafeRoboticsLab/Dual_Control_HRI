classdef Bicycle4D
    % 4D kinematic bicycle model.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-08, Last modified: 2021-11-08
    
    properties
        L           % Wheel base
        ts          % Sampling time (s)
        x_dim       % State dimension
        u_dim       % Control dimension
        dfdx        % Symbolic Jacobian matrix dfdx
        dfdu        % Symbolic Jacobian matrix dfdu                 
    end    
    
    methods
        function obj = Bicycle4D(L, ts)
            % Constructor.
            
            % Static parameters.
            obj.L = L;
            obj.ts = ts;
            obj.x_dim = 4;
            obj.u_dim = 2;
            
            % Define linearized dynamics symbolically.
            x_sym = sym('x', [obj.x_dim 1]);
            u_sym = sym('u', [obj.u_dim 1]);
            
            f = [x_sym(4) * cos(x_sym(3));
                 x_sym(4) * sin(x_sym(3));
                 (x_sym(4) / obj.L) * tan(u_sym(2));
                 u_sym(1)];
             
            obj.dfdx = matlabFunction(jacobian(f,x_sym),'Vars',{x_sym,u_sym});       
            obj.dfdu = matlabFunction(jacobian(f,u_sym),'Vars',{x_sym,u_sym});        
        end
        
        function x_dot = dynamicsODE(obj, x, u)
            % Compute the time derivative of state for a particular x/u.
            % 4D (kinematic) bicycle model. Dynamics are as follows:
            %                          \dot x     = v cos(phi)
            %                          \dot y     = v sin(phi)
            %                          \dot phi   = (v / L) tan(u2)
            %                          \dot v     = u1
            %
            % Dynamics were taken from:
            % https://arxiv.org/abs/1711.03449
            
            x_dot(1,1) = x(4) * cos(x(3));
            x_dot(2,1) = x(4) * sin(x(3));
            x_dot(3,1) = (x(4) / obj.L) * tan(u(2));
            x_dot(4,1) = u(1);
        end
        
        function x = integrate(obj, x0, u, dt)
            % Integrate initial state x0 (applying constant control u) over 
            % a time interval of length dt.
            
            t = 0.0;
            x = x0;
            while t < obj.ts - 1e-8
                % Make sure we don't step past T.
                step = min(dt, obj.ts - t);

                % Use Runge-Kutta order 4 integration. For details please 
                % refer to https://en.wikipedia.org/wiki/Runge-Kutta_methods.
                k1 = step * obj.dynamicsODE(x, u);
                k2 = step * obj.dynamicsODE(x + 0.5 * k1, u);
                k3 = step * obj.dynamicsODE(x + 0.5 * k2, u);
                k4 = step * obj.dynamicsODE(x + k3, u);

                x = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
                t = t + step;
            end
        end
        
        function [Ac, Bc] = linearize(obj, x0, u0)
            % Compute the Jacobian linearization of the dynamics for a 
            % particular state x0 and control u0. Outputs A and B matrices 
            % of a linear system:
            %       \dot x - f(x0, u0) = A (x - x0) + B (u - u0)
            Ac = obj.dfdx(x0, u0);
            Bc = obj.dfdu(x0, u0);
        end
        
        function [Ad, Bd] = linearizeDiscrete(obj, x0, u0)
            % Compute the Jacobian linearization of the dynamics for a 
            % particular state x0 and control u0. Outputs A and B matrices 
            % of a discrete-time linear system:
            %       x(k + 1) - x0 = A (x(k) - x0) + B (u(k) - u0)
            [Ac, Bc] = obj.linearize(x0, u0);
%             sysc = ss(Ac,Bc,eye(obj.x_dim),[]);
%             sysd = c2d(sysc,obj.ts);
%             Ad = sysd.A;
%             Bd = sysd.B;

            % See https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
            % for derivation of discrete-time from continuous time linear 
            % system.
            Ad = expm(Ac * obj.ts);
            Bd = pinv(Ac)*(Ad - eye(obj.x_dim))*Bc;
        end
        
        function P = LQRTerminalCost(obj, xF, Q, R)
            % Compute LQR terminal cost.
            [Ad, Bd] = obj.linearizeDiscrete(xF, [0;0]);
            [~,P,~] = dlqr(Ad,Bd,Q,R);
        end
        
        function P = LQRTerminalCost_no_px(obj, xF, Q, R)
            % Compute LQR terminal cost (no px).
            [Ad, Bd] = obj.linearizeDiscrete(xF, [0;0]);
            [~,P,~] = dlqr(Ad(2:4,2:4),Bd(2:4,:),Q(2:4,2:4),R);
        end
        
        function [uSol, xSol, VSol] = NMPC(obj, x0, xF, N, Q, R, ulb,...
                uub, plb, pub, vlb, vub, planner)
            % Compute an NMPC control.
            
%             yalmip('clear')
            params = planner.params;

            % Define solver settings.
            options = sdpsettings('verbose', 0, 'solver', 'snopt',...
                'usex0', 0, 'cachesolvers', 1);

            %--------------------------------------------------------------
            % Define decision variables
            %--------------------------------------------------------------
            % States.
            x = sdpvar(obj.x_dim, N+1);

            % Inputs.
            u = sdpvar(obj.u_dim, N);

            %--------------------------------------------------------------
            % Objective function
            %--------------------------------------------------------------
            % DLQR-based terminal cost.
            [Ad, Bd] = obj.linearizeDiscrete(xF, [0;0]);
            [~,P,~] = dlqr(Ad(2:4,2:4),Bd(2:4,:),Q(2:4,2:4),R);
            
            objective = (x(2:4,N+1)-xF(2:4))'*P*(x(2:4,N+1)-xF(2:4));
            for i = 1:N
               objective = objective + u(:,i)'*R*u(:,i)+...
                   (x(2:4,i)-xF(2:4))'*Q(2:4,2:4)*(x(2:4,i)-xF(2:4));
            end

            %--------------------------------------------------------------
            % Bounds on the states and inputs
            %--------------------------------------------------------------
            constraint = [];
            % Input constraints.
            for i = 1:N
                constraint = [constraint ulb<=u(:,i)<=uub];
            end

            % State constraints.
            for i = 1:N+1
                constraint = [constraint plb<=x(3,i)<=pub,...    % phi bounds
                              constraint vlb<=x(4,i)<=vub,...    % velocity bounds
                    params.rd_bd_min<=x(2,i)<=params.rd_bd_max]; % road boundaries
            end

            %--------------------------------------------------------------
            % Initial state constraints
            %--------------------------------------------------------------
            constraint = [constraint x(:,1)==x0];

            %--------------------------------------------------------------
            % Dynamics constraints
            %--------------------------------------------------------------
            for i = 1:N
                constraint = [constraint...
                    x(:,i+1)==x(:,i)+obj.ts*obj.dynamicsODE(x(:,i),u(:,i))];
            end

            %--------------------------------------------------------------
            % Solve the MPC problem
            %--------------------------------------------------------------
            status = optimize(constraint, objective, options);
            isSolved = (status.problem==0);
            if ~isSolved
                message = ['solver info: ', status.info ];
                warning(message);
            end

            %--------------------------------------------------------------
            % Return values
            %--------------------------------------------------------------
            xSol = value(x);
            uSol = value(u);
            VSol = value(objective);
        end
    end % end methods
end % end class

