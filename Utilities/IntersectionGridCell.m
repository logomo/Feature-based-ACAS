classdef IntersectionGridCell<handle
    %INTERSECTIONGRIDCELL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cell;               %cellReference
        position;           %position of reach reference
        time;               %reach time reference
        probability;
    end
    
    methods
        function obj=IntersectionGridCell(cell,position,time);
            obj.cell = cell;
            obj.position = position;
            obj.time = time;
        end
    end
    
end

