classdef tree
    % Class for a scenario tree.
    %   Author: Haimin Hu
    %   Created: 2021-12-11, Last modified: 2021-12-11
    
    properties
        Layer       % Index set of nodes in each layer.
        N           % Node set.
        num_node    % Number of nodes.
        planner     % The planner object.
    end
    
    methods
        function obj = tree(root_node, planner)
            % Constructor for the scenario tree.
            obj.Layer = cell(1,100);
            obj.Layer{1} = 1;
            if isnan(root_node.idx)
                root_node.idx = 1;
            end
            if root_node.t ~= 1
                root_node.t = 1;
            end
            obj.N = root_node;
            obj.num_node = 1;
            obj.planner = planner;
        end
        
        
        function obj = addNewNode(obj, node)
            % Adds a new node to the tree.
            obj.num_node = obj.num_node + 1;
            node.idx = obj.num_node;
            obj.N = [obj.N node];
            
            % Update Layer.
            obj.Layer{node.t} = [obj.Layer{node.t} node.idx];
            
            % Sanity check.
            if length(obj.N) ~= node.idx
                error('Node index incorrect! Check for bugs!')
            end
        end

        
        function obj = constructTree(obj, Nd, Ne, M_set, extraArg, verbose)
            % Constructs a scenario tree with Nd dual steps and Ne
            % exploitation steps.

            % ------- Begin tree construction -------
%             if verbose
%                 disp('Constructing the scenario tree...')
%             end
            
            % ------- Dual control steps -------
            num_node_dual = 0;
            for k = 1:Nd          
                for idx = obj.Layer{k}
                    new_node_vec = obj.N(idx).branch(M_set, extraArg);
                    for new_node = new_node_vec
                        obj = obj.addNewNode(new_node);
                        num_node_dual = num_node_dual + 1;
                    end
                end
            end
            
            % ------- Exploitation steps -------
            num_node_expl = 0;
            for k = Nd+1:Nd+Ne
                for idx = obj.Layer{k}
                    new_node = obj.N(idx).extend();
                    obj = obj.addNewNode(new_node);
                    num_node_expl = num_node_expl + 1;
                end
            end
            
            % ------- End tree construction -------
          
            % Report construction results.
%             if verbose
%                 disp('Tree construction completed!')
%                 for k = 1:Nd+Ne+1
%                     disp(['Layer #', num2str(k), ': ',...
%                         num2str(length(obj.Layer{k})), ' node(s)'])
%                 end
%                 disp(['Number of dual control node(s): ',...
%                     num2str(num_node_dual)])
%                 disp(['Number of exploitation node(s): ',...
%                     num2str(num_node_expl)])
%             end
        end % end constructTree
    end % end methods
end % end class

