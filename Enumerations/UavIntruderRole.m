classdef UavIntruderRole<uint32
    %UavIntruderRole The role of the UAS in the Collision Case
    
    enumeration
        RightOfTheWay(1),   %Has ROA, continue on heading
        Overtaking(2),      %Is overtaking other UAS
        Converging(3),      %Is converging to avoid other UAS
        Roundabouting(4),   %Is joining/joined virtual roundabout
        Emergencing(5)      %Is in emergency Avoidance mode
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
    
end

