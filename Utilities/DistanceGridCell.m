classdef DistanceGridCell<handle
    %DISTANCEGRIDCELL Helper class containing additional cell check data
    
    properties
        cell;           % Cell reference
        checkDistance;  % check distance [m]
        realDistance;   % real distance [m]
        isDirect;       % direct intersection idicator [boolean]
        isTolerated;    % tolerated calculation range [boolean]
    end
    
    methods
        function obj=DistanceGridCell(cell,checkDistance,realDistance,isDirect,isTolerated)
            %Fill the structure with data
            obj.cell = cell;
            obj.checkDistance = checkDistance;
            obj.realDistance = realDistance;
            obj.isDirect = isDirect;
            obj.isTolerated = isTolerated;
        end
    end
    
end

