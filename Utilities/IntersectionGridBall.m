classdef IntersectionGridBall<handle
    %INTERSECTIONGRIDBALL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        center
        coating
    end
    
    methods
        function obj=IntersectionGridBall(center,coating)
            obj.center = center;
            obj.coating = coating;
        end
    end
    
end

