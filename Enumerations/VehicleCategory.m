classdef VehicleCategory<uint32
    %STATISTICTYPE Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        MannedDistress(1),
        Balloon(2),
        Gliver(3),
        Towing(4),
        Airship(5),
        MannedGeneral(6),
        UAVAutonomous(7),
        UAVPilloted(8)
    end
    
    methods(Static)
        function r=toString(member)
            r=string(char(member));
        end
    end
end

