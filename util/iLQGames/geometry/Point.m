classdef Point
    % Point class for 2D points.
    %   Author: Haimin Hu
    %   Reference: ilqgames by David Fridovich-Keil 
    %   Created: 2021-11-08, Last modified: 2021-11-08

    properties
        x
        y
    end
    
    methods
        function obj = Point(x, y)
            % Constructor
            obj.x = x;
            obj.y = y;
        end

        function new_Point = plus(obj, rhs)
            new_Point = Point(obj.x + rhs.x, obj.y + rhs.y);
        end

        function new_Point = minus(obj, rhs)
            new_Point = Point(obj.x - rhs.x, obj.y - rhs.y);
        end

        function new_Point = mtimes(obj, rhs)
            new_Point = Point(obj.x * rhs, obj.y * rhs);
        end

        function new_Point = mrdivide(obj, rhs)
            new_Point = Point(obj.x / rhs, obj.y / rhs);
        end

        function out = norm_squared(obj)
            out = obj.x^2 + obj.y^2;
        end

        function out = norm(obj)
            out = sqrt(obj.norm_squared());
        end
    end % end methods
end % end class

