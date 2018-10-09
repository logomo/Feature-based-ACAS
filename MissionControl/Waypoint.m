classdef Waypoint<handle
    %WAYPOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position=[0;0;0];
    end
    
    methods
        function obj=Waypoint(x,y,z)
            if nargin ==1
                obj.position=x;
            end
            if nargin ==3
                obj.position=[x;y;z];
            end
        end
    end
    
end

