classdef RuleHeadOnApproachManeuver<AbstractRule
    %RuleHeadOnApproachManeuver Head on situation resolution for single UAS
    
    
    properties
       vehicleId,           %UAS UQ ID
       collisionCase,       %Participating collision case reference
       collisionPoint,      %3D GCF collision point
       missionControl,      %UAS mission control reference
       avoidanceGrid,       %UAS avoidance grid
       velocity,            %UAS velocity at the moment of detection
       reachSet,            %UAS active reach set reference
       position,            %UAS actual position
       orientation,         %UAS actual orientation
       collisionCategory,   %Collision role of UAS
       avoidanceRole,       %ROW categorization
       safetyMargin,        %Separation distance
    end
    
    methods
        function obj = RuleHeadOnApproachManeuver(context,jointPointCode,ruleCode)
            %Create rule instance based on context
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            %[Override] Parse rule context to internal datastructure
            priorFlag=parseContext@AbstractRule(obj);
            % Additional parse context functionality here
            % START
            obj.vehicleId=obj.context('vehicleId');
            obj.collisionCase=obj.context('collisionCase');
            obj.collisionPoint=obj.context('collisionPoint');
            obj.missionControl=obj.context('missionControl');
            obj.avoidanceGrid=obj.context('avoidanceGrid');
            obj.velocity=obj.context('velocity');
            obj.reachSet=obj.context('reachSet');
            obj.position=obj.context('position');
            obj.orientation=obj.context('orientation');
            obj.collisionCategory=obj.context('collisionCategory');
            obj.avoidanceRole=obj.context('avoidanceRole');
            obj.safetyMargin=obj.context('safetyMargin');
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            %[Override] test condition sanity check
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            %[Override] rule body applicaiton
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            % Node application function
            % test node - move to the right
            %node=root.leafs(3);
            %[flagDistanceFeasibility,flagHeadingFeasibility] = node.calculateFlagsNode(mc,vel,pos,or,collisionPoint,ruleSafetyMargin);
            nodePassed=obj.reachSet.calculateOperatibleSpace(...
                                    obj.missionControl,...
                                    obj.velocity,...
                                    obj.position,obj.orientation,...
                                    obj.collisionPoint,...
                                    obj.safetyMargin);

            %% Rule application
            nodeDisabled = obj.reachSet.applyOperatibleSpace;
            
            
            %obj.avoidanceGrid.recalculate;
            functionFlag=true;
            %obj.missionControl.plotGridSlice(obj.avoidanceGrid,StatisticType.Reachability,2)
            % END
            r= priorFlag && functionFlag;
        end
    end
end

