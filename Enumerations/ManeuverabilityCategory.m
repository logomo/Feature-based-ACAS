classdef ManeuverabilityCategory<uint32
    %STATISTICTYPE Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
       AltitudeControl(1),
       FullNoPropulsion(2),
       FullPropulsionGliding(3),
       FullPropulsionVTOL(4),
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
    
end

