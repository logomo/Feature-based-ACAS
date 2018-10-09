classdef CollisionCategory<uint32
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        HeadOnApproach(1)
        Overtaking(2)
        Converging(3)
        Unknown(4)
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

