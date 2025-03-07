classdef TrajectoryPoint
    properties
        position
        orientation
    end
    methods
        function obj = TrajectoryPoint()
            obj.position = zeros(1, 3);
            obj.orientation = zeros(1, 3);
        end
    end
end
