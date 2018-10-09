classdef DistanceGridCell<handle
    %DISTANCEGRIDCELL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cell;
        checkDistance;
        realDistance;
        isDirect;
        isTolerated;
    end
    
    methods
        function obj=DistanceGridCell(cell,checkDistance,realDistance,isDirect,isTolerated)
            obj.cell = cell;
            obj.checkDistance = checkDistance;
            obj.realDistance = realDistance;
            obj.isDirect = isDirect;
            obj.isTolerated = isTolerated;
        end
    end
    
end

