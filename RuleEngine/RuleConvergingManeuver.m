classdef RuleConvergingManeuver<AbstractRule
    %RuleConvergingManeuver Converging manuever enforcement
    
    
    properties
       vehicleId,           %UAS UQ ID    
       collisionCase,       %Related collision case reference
       collisionPoint,      %3D collision point GFC
       missionControl,      %UAS Mission Control object reference
       avoidanceGrid,       %UAS avoidance grid reference
       velocity,            %UAS velocity
       reachSet,            %UAS active reach set reference
       position,            %UAS position at  the moment of detection
       orientation,         %UAS orientation at the moment of detection
       collisionCategory,   %Collision category
       avoidanceRole,       %UAS ROW category
       safetyMargin,        %Enforced minimal distance to adversity
    end
    
    methods
        function obj = RuleConvergingManeuver(context,jointPointCode,ruleCode)
            %Rule initialization
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            %Context parsing
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
            %Integrity check
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            %Rule body application
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            % Safety margin fix adding the speed into account
            % vehicle velocity * simulation step (parameter for decision time - yeah bad name)
            obj.safetyMargin=obj.safetyMargin+ 4*(obj.velocity*Cmnf.simStep);
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

