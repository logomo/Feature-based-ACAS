classdef StatisticType<uint32
    %STATISTICTYPE Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        Reachability(1),
        Visibility(2),
        Obstacle(3),
        Decision(4),
        Feasibility(5),
        GraphObstacle(6),
        RuleInvocation(7),
        TrimmedReach(8),
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

