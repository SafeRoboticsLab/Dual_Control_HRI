classdef node
    % Class for a node in the scenario tree.
    %   Author: Haimin Hu
    %   Created: 2021-12-11, Last modified: 2021-12-11
    
    properties
        idx                 % Node index.
        pre_node_idx        % Index of the pre node.
        t                   % Time step.
        xR                  % Robot's state.
        xH                  % Human's state.
        uR	                % Robot's control.
        uH                  % Human's control.
        uH_nom              % Human's nominal control (rational).
        eps                 % Soft constraint slack variables.
        theta_sample        % Sample of theta (transformed).
        theta_sample_sn     % Sample of theta (sampled from a std. normal).
        theta_distr         % Distribution of theta.
        theta_distr_M_ws    % Distribution of theta given M (for warmstart only).
        M_sample            % Sample of M.
        M_sample_prob       % Probability of sample of M.
        M_distr             % Distribution of M.
        path_prob           % Path transition probability of the node.
        xR_dim              % Dimension of robot's state.
        xH_dim              % Dimension of human's state.
        uR_dim              % Dimension of robot's control.
        uH_dim              % Dimension of human's control.
        theta_dim           % Dimension of theta.
        M_dim               % Dimension of M.
        xR_dims             % Indices of robot's state.
        xH_dims             % Indices of human's state.
        uR_dims             % Indices of robot's control.
        uH_dims             % Indices of human's control.
    end
    
    methods
        function obj = node(idx, pre_node_idx, t, path_prob,...
                theta_sample_sn, M_sample, xR_dim, xH_dim, uR_dim, uH_dim,...
                theta_dim, M_dim, xR_dims, xH_dims, uR_dims, uH_dims)
            % Constructor for the node class.
            obj.idx = idx;
            obj.pre_node_idx = pre_node_idx;
            obj.t   = t;
         	obj.xR  = sdpvar(xR_dim,1);
            obj.xH  = sdpvar(xH_dim,1);
            obj.uR  = sdpvar(uR_dim,1);
            obj.uH  = sdpvar(uH_dim,1);
            obj.uH_nom  = sdpvar(uH_dim,1);
            obj.eps.RB_R = sdpvar(1,1);
            obj.eps.RB_H = sdpvar(1,1);
            obj.eps.CA   = sdpvar(1,1);
            obj.theta_distr.mu = sdpvar(theta_dim,1);
            obj.theta_distr.covar = sdpvar(theta_dim,theta_dim,'diagonal');
            obj.path_prob = path_prob;
            obj.theta_sample = sdpvar(theta_dim,1);
            obj.theta_sample_sn = theta_sample_sn;
            obj.M_sample = M_sample;
            obj.M_distr = sdpvar(M_dim,1);
            obj.xR_dim = xR_dim;
            obj.xH_dim = xH_dim;
            obj.uR_dim = uR_dim;
            obj.uH_dim = uH_dim;
            obj.theta_dim = theta_dim;
            obj.M_dim = M_dim;
            obj.xR_dims = xR_dims;
            obj.xH_dims = xH_dims;
            obj.uR_dims = uR_dims;
            obj.uH_dims = uH_dims;
        end
        
        
        function node_vec = branch(obj, M_set, extraArg)
            % Branches from this node to obtain descendants (for dual
            % control steps).
            node_vec = [];
            
            % Generate samples of theta from a std. normal (binary).
            sample_sn = extraArg.theta_sample_sn;
            theta_sample_sn_1 = [sample_sn;-sample_sn]; theta_prob_1 = 1/2;
            theta_sample_sn_2 = [-sample_sn;sample_sn]; theta_prob_2 = 1/2;
            theta_sample_sn_vec = [theta_sample_sn_1 theta_sample_sn_2];
            theta_prob_vec = [theta_prob_1 theta_prob_2];
            if obj.t >= extraArg.N_M + 1
                M_set = obj.M_sample;
            end

            for i = 1:numel(theta_prob_vec)
              % Create new nodes.
                for M = M_set
                    new_node_path_prob = obj.path_prob * theta_prob_vec(i);

                    new_node = node([], obj.idx, obj.t + 1,...
                        new_node_path_prob, theta_sample_sn_vec(:,i), M,...
                        obj.xR_dim, obj.xH_dim, obj.uR_dim,  obj.uH_dim,...
                        obj.theta_dim, obj.M_dim, obj.xR_dims,...
                        obj.xH_dims, obj.uR_dims, obj.uH_dims);

                    % Update the descendant node vector.
                    node_vec = [node_vec new_node];
                end
            end
        end
        
        
        function new_node = extend(obj)
            % Extends this node to obtain one descendant (for exploitation
            % steps).
            theta_sample_sn_1 = [0; 0]; % mean of the std. normal.
            
            new_node = node([], obj.idx, obj.t + 1, obj.path_prob,...
                theta_sample_sn_1, obj.M_sample, obj.xR_dim, obj.xH_dim,...
                obj.uR_dim, obj.uH_dim, obj.theta_dim, obj.M_dim,...
                obj.xR_dims, obj.xH_dims, obj.uR_dims, obj.uH_dims);
        end
    end % end methods
end % end class

