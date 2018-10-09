classdef UavIntruderRole<uint32
    %UavIntruderRole Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        RightOfTheWay(1),
        Overtaking(2),
        Converging(3),
        Roundabouting(4),
        Emergencing(5)
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
    
end

