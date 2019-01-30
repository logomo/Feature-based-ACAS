classdef IntersectionGridCell<handle
    %INTERSECTIONGRIDCELL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cell;               %cellReference
        position;           %position of reach reference
        time;               %reach time reference
        probability;        %rate of intersection [0,1]
    end
    
    methods
        function obj=IntersectionGridCell(cell,position,time)
            %Constructor accepting calculated data
            obj.cell = cell;
            obj.position = position;
            obj.time = time;
        end
    end
    
end

