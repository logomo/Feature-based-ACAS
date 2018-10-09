classdef RuleCode<uint32    
    enumeration
        PriorRulesOfAir(0),
        PostRulesOfAir(1),
        ConvergingManeuver(2),
        HeadOnApproachManeuver(3),
        OvertakeManevuer(4),
        TestRule(5),
        CollisionCaseResulution(6),
    end   
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    end
end

