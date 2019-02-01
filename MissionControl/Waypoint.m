classdef Waypoint<handle
    %WAYPOINT coating function to represent waypoinnt as object
    
    properties
        position=[0;0;0];   %Local position
    end
    
    methods
        function obj=Waypoint(x,y,z)
            %Waypoint constructor 
            %   x - GCF x
            %   y - GCF y
            %   z - GCF z
            if nargin ==1
                obj.position=x;
            end
            if nargin ==3
                obj.position=[x;y;z];
            end
        end
    end
    
end

