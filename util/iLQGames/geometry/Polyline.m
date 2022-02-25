classdef Polyline
    % Polyline class to represent piecewise linear path in 2D.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-08, Last modified: 2021-11-08
    
    properties
        points
    end
    
    methods
        function obj = Polyline(points)
            % Initialize from a list of points. Keeps only a reference to 
            % input list.
            obj.points = points;
        end
        
        function out = signed_distance_to(obj, point)
            % Compute signed distance from this polyline to the given point.
            % Sign convention is positive to the right and negative to the left, e.g.:
            %                                 *
            %                                 |
            %            negative             |             positive
            %                                 |
            %                                 |
            %                                 *
            % NOTE: for now, we'll just implement this with a naive linear search.
            % In future, if we want to optimize at all we can pass in a guess of
            % which index we expect the closest point to be close to.
            best_signed_distance = 1e8;
            for ii = 1:length(obj.points)-1
                segment = LineSegment(obj.points(ii), obj.points(ii + 1));
                signed_distance = segment.signed_distance_to(point);

                if abs(signed_distance) < abs(best_signed_distance)
                    best_signed_distance = signed_distance;
                end
            end
            out = best_signed_distance;
        end
        
    end % end methods
end % end class

