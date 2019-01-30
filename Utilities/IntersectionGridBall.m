classdef IntersectionGridBall<handle
    %INTERSECTIONGRIDBALL Intersection set for grid ball checl
    
    
    properties
        center          %obstacle center point
        coating         %obstacle coating points
    end
    
    methods
        function obj=IntersectionGridBall(center,coating)
            %Constructor accepting calculated data
            obj.center = center;
            obj.coating = coating;
        end
    end
    
end

