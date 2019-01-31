classdef VehicleCategory<uint32
    %VehicleCategory used in current Rules of the Air implementation
    
    enumeration
        MannedDistress(1),      %Any manned avitation in distress has the right of the way
        Balloon(2),             %Vertical manuever control only (manned aviation only)
        Gliver(3),              %Full manuevurability no propulsion (manned aviation only)
        Towing(4),              %The propulsion full control is towing/fueling a load/plane (manned aviation only)
        Airship(5),             %The airships have full manuevurability but super long reaction time and very low cruising speed.
        MannedGeneral(6),       %Any manned aviation not belonging to previously mentioned catefories - all planes/copters
        UAVAutonomous(7),       %Any kind of Autonomous craft (proposal)
        UAVPilloted(8)          %Any kind of RPAS (proposal)
    end
    
    methods(Static)
        function r=toString(member)
            r=string(char(member));
        end
    end
end

