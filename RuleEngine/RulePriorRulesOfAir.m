classdef RulePriorRulesOfAir<AbstractRule
    %TESTRULE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        missionControl
        vehicleId
        vehicleName
        collisionCases
        activeCollisionCases
        avoidanceGrid
        navigationGrid
    end
    
    methods
        function obj = RulePriorRulesOfAir(context,jointPointCode,ruleCode)
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
            
            obj.collisionCases= obj.utmControl.selectCollisionCases(obj.vehicleId);
            
            if ~isempty(obj.collisionCases)
                obj.activeCollisionCases=[];
                for k=1:length(obj.collisionCases)
                    ccCandidate=obj.collisionCases(k);
                    if ccCandidate.isOpenCase
                        obj.activeCollisionCases=[obj.activeCollisionCases,ccCandidate];
                    end
                end
            end
            functionFlag=~isempty(obj.collisionCases);
            
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=~isempty(obj.activeCollisionCases);
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            params=obj.missionControl.reParams;
            params('activeCollisionCases')=obj.activeCollisionCases;
            obj.missionControl.ruleFlag=1;
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

