classdef StatisticType<uint32
    %STATISTICTYPE Type of the statistics to be tracked plot
    
    enumeration
        Reachability(1),        %Reachibility in cells/trajectories
        Visibility(2),          %Visibility in cells/space portions
        Obstacle(3),            %Final static obstacle probability in cells
        Decision(4),            %Decision points and frames along UAS trajectories
        Feasibility(5),         %[Deprecated] use Reachibility
        GraphObstacle(6),       %[Volatile] use only with \Djikstra representaiton
        RuleInvocation(7),      %[Volatile] use only with \RuleEngine active in Mission Control
        TrimmedReach(8),        %Reachibility in cell/trajectories but only over treshold, unfeasible parts are not displayed
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

