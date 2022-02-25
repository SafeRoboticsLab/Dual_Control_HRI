classdef ProductMultiPlayerDynamicalSystem
    % Implements a multiplayer dynamical system who's dynamics decompose 
    % into a Cartesian product of single-player dynamical systems.
	%   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-10, Last modified: 2021-11-10
    
    properties
        ts          % Sampling time (s)
        x_dim       % State dimension
        x_dims      % Cell of index lists of states of each player
        u_dim       % Control dimension
        u_dims      % Cell of index lists of controls of each player
        num_players % Number of players
        subsystems  % Cell of DynamicalSystem objects
        is_linear   % Flag indicating if all subsystems are linear
        Ac_lin      % Continuous-time A matrix (only for linear case)
        Bc_lin      % Continuous-time B matrix (only for linear case)
        Ad_lin      % Discrete-time A matrix (only for linear case)
        Bd_lin      % Discrete-time B matrix (only for linear case)
    end
    
    methods
        function obj = ProductMultiPlayerDynamicalSystem(ts, x_dim,...
                x_dims, u_dim, u_dims, subsystems, is_linear)
            % Constructor.
            obj.ts = ts;
            obj.x_dim = x_dim;
            obj.x_dims = x_dims;
            obj.u_dim = u_dim;
            obj.u_dims = u_dims;
            obj.num_players = length(subsystems);
            obj.subsystems = subsystems;
            
            if nargin < 7
                obj.is_linear = 0;
            else
                obj.is_linear = is_linear;
            end
            
            if obj.is_linear
                Ac = [];
                Bc = [];
                for i = 1:obj.num_players
                    [Ac_i, Bc_i] = obj.subsystems{i}.linearize([], []);
                    Ac = blkdiag(Ac, Ac_i);
                    Bc = blkdiag(Bc, Bc_i);
                end
                obj.Ac_lin = Ac;
                obj.Bc_lin = Bc;
                sysc = ss(obj.Ac_lin, obj.Bc_lin, eye(obj.x_dim), []);
                sysd = c2d(sysc, obj.ts);
                obj.Ad_lin = sysd.A;
                obj.Bd_lin = sysd.B;
            end
        end
        
        function x = integrate(obj, x0, u, dt)
            % Integrate initial state x0 (applying constant control u) over 
            % a time interval of length dt, using a time discretization of
            % dt.
            x = [];
            for i = 1:obj.num_players
                xi = x0(obj.x_dims{i});
                ui = u(obj.u_dims{i});
                xi = obj.subsystems{i}.integrate(xi, ui, dt);
                x = [x; xi];
            end
        end
        
        function [Ac, Bc] = linearize(obj, x0, u0)
            % Compute the Jacobian linearization of the dynamics.
            Ac = [];
            Bc = [];
            for i = 1:obj.num_players
                x0_i = x0(obj.x_dims{i});
                u0_i = u0(obj.u_dims{i});
                [Ac_i, Bc_i] = obj.subsystems{i}.linearize(x0_i, u0_i);
                Ac = blkdiag(Ac, Ac_i);
                Bc = blkdiag(Bc, Bc_i);
            end
        end
        
        function [Ad, Bd] = linearizeDiscrete(obj, x0, u0)
            % Compute linearization & discretization of the system.
            [Ac, Bc] = obj.linearize(x0, u0);
%             sysc = ss(Ac,Bc,eye(obj.x_dim),[]);
%             sysd = c2d(sysc,obj.ts);
%             Ad = sysd.A;
%             Bd = sysd.B;

            Ad = expm(Ac * obj.ts);
            Bd = pinv(Ac)*(Ad - eye(obj.x_dim))*Bc;
                
%             Ad = eye(obj.x_dim) + Ac*obj.ts;
%             Bd = Bc*obj.ts;
        end
        
        function [Ad, Bd_cell] = linearizeDiscrete_B_cell(obj, x0, u0)
            % Compute linearization & discretization of the system, output 
            % Bd as a cell.
            if obj.is_linear
                Ad = obj.Ad_lin;
                Bd_cell = {};
                idx = 1;
                for i = 1:obj.num_players
                    Bd_cell{end+1} =...
                        obj.Bd_lin(:,idx:idx+obj.subsystems{i}.u_dim-1);
                    idx = idx+obj.subsystems{i}.u_dim;
                end
            else
                [Ad, Bd] = obj.linearizeDiscrete(x0, u0);
                Bd_cell = {};
                idx = 1;
                for i = 1:obj.num_players
                    Bd_cell{end+1} =...
                        Bd(:,idx:idx+obj.subsystems{i}.u_dim-1);
                    idx = idx+obj.subsystems{i}.u_dim;
                end
            end
        end
    end % end methods
end % end class

