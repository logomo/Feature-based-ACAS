classdef ManeuverabilityCategory<uint32
    %ManeuverabilityCategory This defines manuevurability of the air trafic attendant used in Collision Case resolution process
    
    enumeration
       AltitudeControl(1),           %Craft supports going up or down 
       FullNoPropulsion(2),          %Craft supports full horizontal/vertical manuevering, whitout own source of propulsion
       FullPropulsionGliding(3),     %Craft supports full horizontal/vertical manuevering, own propulsion, non stop movement
       FullPropulsionVTOL(4),        %Craft supports full horizontal/vertical manuevering, own propulsion, stop movement enabled 
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
    
end

