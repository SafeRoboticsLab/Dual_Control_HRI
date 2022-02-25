classdef LineSegment
    % Class for 2D line segments.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-08, Last modified: 2021-11-08
    
    properties
        p1
        p2
    end
    
    methods
        function obj = LineSegment(p1, p2)
            % Constructor
            obj.p1 = p1;
            obj.p2 = p2;
        end
        
        function out = len(obj)
            out = norm(obj.p1 - obj.p2);
        end

        function out = signed_distance_to(obj, point)
            % Compute signed distance to other point.
            % Sign convention is positive to the right and negative to the left, e.g.:
            %                                 *
            %                                 |
            %            negative             |             positive
            %                                 |
            %                                 |
            %                                 *
            
            % Vector from p1 to query.
            relative = point - obj.p1;

            % Compute the unit direction of this line segment.
            direction = obj.p2 - obj.p1;
            direction = direction / norm(direction);

            % Find signed length of projection and of cross product.
            projection = relative.x * direction.x + relative.y * direction.y;
            cross = relative.x * direction.y - direction.x * relative.y;
            if cross >= 0.0
                cross_sign = 1.0;
            else
                cross_sign = -1.0;
            end

            if projection < 0.0
                % Query lies behind this line segment, so closest distance 
                % will be from p1.
                out = cross_sign * norm(relative);
            elseif projection > obj.len()
                % Closest distance will be to p2.
                out = cross_sign * norm(obj.p2 - point);
            else
                out = cross;
            end
        end
    end % end methods
end % end class

