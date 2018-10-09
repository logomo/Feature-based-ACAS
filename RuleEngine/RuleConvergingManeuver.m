classdef RuleConvergingManeuver<AbstractRule
    %TESTRULE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       vehicleId,
       collisionCase,
       collisionPoint,
       missionControl,
       avoidanceGrid,
       velocity,
       reachSet,
       position,
       orientation,
       collisionCategory,
       avoidanceRole,
       safetyMargin,
    end
    
    methods
        function obj = RuleConvergingManeuver(context,jointPointCode,ruleCode)
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
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
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
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

