classdef RuleStaticWeather<AbstractRule
    %RuleStaticWeather [DEPRECATED] 
    %   Weather implemented as soft/hard constraints
    
    properties
        
    end
    
    methods
        function obj = RuleStaticWeather(context,jointPointCode,ruleCode)
            %[DEPRECATED]
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            %[DEPRECATED]
            priorFlag=parseContext@AbstractRule(obj);
            % Additional parse context functionality here
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            %[DEPRECATED]
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            %[DEPRECATED]
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

