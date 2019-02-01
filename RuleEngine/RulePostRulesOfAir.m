classdef RulePostRulesOfAir<AbstractRule
    %RulePostRulesOfAir result collection rule
    %   Gets all spoils and constraints
    
    properties
        missionControl          %UAS mission control object reference
        vehicleId               %UAS UQ ID
        vehicleName             %UAS name
        activeCollisionCases    %Listiong of active collision cases related to UAS
        avoidanceGrid           %avoidance grid reference
        navigationGrid          %navigation grid reference
    end
    
    methods
        function obj = RulePostRulesOfAir(context,jointPointCode,ruleCode)
            %Rule instance initialization
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            %[Overide] Parse the rule context
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
            %[Override] sanity check of the rule data
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=~isempty(obj.activeCollisionCases);
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            %[Override] Rule body implementation
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            obj.missionControl.reParams('activeCollisionCases')=[];
            obj.missionControl.ruleFlag=false;
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

