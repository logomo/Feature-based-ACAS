classdef RuleEngine<LoggableObject
    %RULEENGINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ruleList;
    end
    
    methods
        
        %% Constructor
        function obj = RuleEngine()
            %RULEENGINE Construct an instance of this class
            %   Detailed explanation goes here
            obj.ruleList = containers.Map;
        end
        %% Activate Rule in Rule engine
        function r=activateRule(obj,jointPointCode,ruleCode)
           jpsc=RuleJointPoint.toString(jointPointCode);
           rsc=RuleCode.toString(ruleCode);
           
           if obj.ruleList.isKey(jpsc)
               jointPointMap = obj.ruleList(jpsc);
           else
               jointPointMap = containers.Map;
               obj.ruleList(jpsc)=jointPointMap;
           end
           %activateRule
           jointPointMap(rsc)=ruleCode;
           obj.ruleList(jpsc)=jointPointMap;
           r=0;
        end
        %% activate rules
        function r=activateRules(obj,jointPointCode,ruleCodes)
            for ruleCode=ruleCodes
                obj.activateRule(jointPointCode,ruleCode);
            end
            r=0;
        end
        %% deactivate rule
        function r=deactiveRule(obj,jointPointCode,ruleCode)
           jpsc=RuleJointPoint.toString(jointPointCode);
           rsc=RuleCode.toString(ruleCode);
           if obj.ruleList.isKey(jpsc)
               jointPointMap = obj.ruleList(jpsc);
           else
               jointPointMap = containers.Map;
           end
           %deactivateRule
           jointPointMap(rsc)=-1;
           obj.ruleList(jpsc)=jointPointMap;
           r=0;
        end
        
        %% deactivate rules
        function r=deactivateRules(obj,jointPointCode,ruleCodes)
            for ruleCode=ruleCodes
                obj.deactiveRule(jointPointCode,ruleCode);
            end
            r=0;
        end
        %% Get rule codes for joint Point
        function r=getRuleCodes(obj,jointPoint)
            jpsc=RuleJointPoint.toString(jointPoint);
            if ~obj.ruleList.isKey(jpsc)
                r=[];
                return;
            end
            jpmap=obj.ruleList(jpsc);
            r=[];
            keys=jpmap.keys;
            for k=1:length(keys)
                value = jpmap(keys{k});
                if value ~= -1
                    r=[r,value];
                end
            end
        end
        
    end
    %% Static invoke method
    methods(Static)
        function r=invoke(rullable,jointPoint)
            % Assumption rullable is RullableObject class implementation
            engine=rullable.getRuleEngine;
            % Sanity check if there is anything to invoke (Rule engine initialized on object)
            if engine == 0 
                return 
            end
            % Get all rule codes
            ruleCodes=engine.getRuleCodes(jointPoint);
            % Load rule context
            context=rullable.getContextRuleEngine;
            % invoke Rules
            for ruleCode=ruleCodes
                %TODO
                invokeFlag=RuleEngine.invokeRule(context,jointPoint,ruleCode);
                if ~invokeFlag
                    Cmnf.logc(rullable,['-- FAILED -- Rule invocation: ',...
                                        RuleCode.toString(ruleCode),...
                                        ', at joint point: ',...
                                        RuleJointPoint.toString(jointPoint)]);
                end
            end
        end
        
        %% InvokeRule
        function r=invokeRule(context,jointPointCode,ruleCode)
            rule = 0;
            
            %Simplerule creation
            if ruleCode == RuleCode.TestRule
                rule=TestRule(context,jointPointCode,ruleCode);
            end
            
            %PriorRulesOfAir creation
            if ruleCode == RuleCode.PriorRulesOfAir
                rule=RulePriorRulesOfAir(context,jointPointCode,ruleCode);
            end
            
            %PostRulesOfAir creation
            if ruleCode == RuleCode.PostRulesOfAir
                rule=RulePostRulesOfAir(context,jointPointCode,ruleCode);
            end
            
            %CollisionCaseResulution creation
            if ruleCode == RuleCode.CollisionCaseResulution
                rule=RuleCollisionCaseResulution(context,jointPointCode,ruleCode);
            end
            
            %Converging Maneuver application
            if ruleCode ==RuleCode.ConvergingManeuver
                rule=RuleConvergingManeuver(context,jointPointCode,ruleCode);
            end
            
            %Head On approach maneuver
            if ruleCode == RuleCode.HeadOnApproachManeuver
                rule=RuleHeadOnApproachManeuver(context,jointPointCode,ruleCode);
            end
            
            %Overtake maneuver
            if ruleCode == RuleCode.OvertakeManevuer
                rule=RuleOvertakeManevuer(context,jointPointCode,ruleCode);
            end
            
            % test if valid rule is fetched
            if rule ~= 0
                rule.runRule;
                r=true;
            else
                r=false;
            end
        end
    end
end

