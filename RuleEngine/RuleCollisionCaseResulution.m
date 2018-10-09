classdef RuleCollisionCaseResulution<AbstractRule
    %TESTRULE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        missionControl
        vehicleId
        vehicleName
        activeCollisionCases
        participationCases
        avoidanceGrid
        navigationGrid
    end
    
    methods
        function obj = RuleCollisionCaseResulution(context,jointPointCode,ruleCode)
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            priorFlag=parseContext@AbstractRule(obj);
            % Additional parse context functionality here
            % START
            obj.missionControl=obj.context('missionControl');
            
            obj.vehicleId=obj.missionControl.vehicleId;
            
            obj.vehicleName=obj.missionControl.vehicleName;
            
            obj.avoidanceGrid=obj.context('avoidanceGrid'); 
            
            obj.navigationGrid=obj.context('navigationGrid'); 
            if obj.missionControl.reParams.isKey('activeCollisionCases')
                obj.activeCollisionCases =obj.missionControl.reParams('activeCollisionCases');
                functionFlag=true;
            else
                functionFlag=false;
            end
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            obj.participationCases=[];
            vehicleId=obj.vehicleId;
            for k=1:length(obj.activeCollisionCases)
                acc=obj.activeCollisionCases(k);
                acc=acc.headCollisionCase;
                if acc.firstId==vehicleId && acc.firstAvoidanceRole ~= UavIntruderRole.RightOfTheWay
                   obj.participationCases=[obj.participationCases,acc];
                end
                if acc.secondId==vehicleId && acc.secondAvoidanceRole ~= UavIntruderRole.RightOfTheWay
                   obj.participationCases=[obj.participationCases,acc];
                end
            end
            functionFlag=~isempty(obj.participationCases);
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            for k=1:length(obj.participationCases)
                cc=obj.participationCases(k);
                collisionPoint=cc.headCollisionCase.collisionPoint;
                %collisionPoint=cc.collisionPoint;
                missionControl=obj.missionControl;
                avoidanceGrid=missionControl.avoidanceGrid;
                velocity=missionControl.vehicleActualVelocity;
                reachSet=avoidanceGrid.reachSet;
                posOr=missionControl.vehicle.getActualPositionOrientation;
                position=posOr(1:3);
                orientation=posOr(4:6);
                collisionCategory=cc.headCollisionCase.collisionCategory;
                safetyMargin=cc.headCollisionCase.ruleSafetyMargin;
                if obj.vehicleId == cc.firstId
                    avoidanceRole=cc.firstAvoidanceRole;
                else
                    avoidanceRole=cc.secondAvoidanceRole;
                end
                manContext=obj.utmControl.copyContextRuleEngine;
                manContext('collisionCase')=cc;
                manContext('vehicleId')=obj.vehicleId;
                manContext('collisionPoint')=collisionPoint;
                manContext('missionControl')=missionControl;
                manContext('avoidanceGrid')=avoidanceGrid;
                manContext('velocity')=velocity;
                manContext('reachSet')=reachSet;
                manContext('position')=position;
                manContext('orientation')=orientation;
                manContext('collisionCategory')=collisionCategory;
                manContext('avoidanceRole')=avoidanceRole;
                manContext('safetyMargin')=safetyMargin;
                if collisionCategory==CollisionCategory.Converging
                    RuleEngine.invokeRule(manContext,obj.jointPointCode,RuleCode.ConvergingManeuver);
                end
                if collisionCategory==CollisionCategory.HeadOnApproach
                    RuleEngine.invokeRule(manContext,obj.jointPointCode,RuleCode.HeadOnApproachManeuver);
                end
                if collisionCategory==CollisionCategory.Overtaking
                    RuleEngine.invokeRule(manContext,obj.jointPointCode,RuleCode.OvertakeManevuer);
                end
                if collisionCategory==CollisionCategory.Unknown
                    %TODO what now ? intruder propagation, wait one turn ?
                    %Rules are void in this case 
                end
            end
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

