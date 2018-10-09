classdef RuleJointPoint<uint32    
    enumeration
        MissionControlRunOnce(0),
        MissionControlNextWaypoint(1),
        UTMControlRunMission(2),
        TestJointPoint(3),
        MissionControlAfterRun(4),
        MissionControlCollisionCaseSolution(5),
    end   
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

