classdef TestRule<AbstractRule
    %TESTRULE Template for rule instance
    %   Copy->Paste change START-END content
    
    properties
        
    end
    
    methods
        function obj = TestRule(context,jointPointCode,ruleCode)
            %[TEMPLATE] constructor template
            obj = obj@AbstractRule(context,jointPointCode,ruleCode);
            %additional init function ality here
            % START
            % END
        end

        function r=parseContext(obj)
            %[TEMPLATE] parse context template
            priorFlag=parseContext@AbstractRule(obj);
            % Additional parse context functionality here
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=testCondition(obj)
            %[TEMPLATE] sanity check template
            priorFlag=testCondition@AbstractRule(obj);
            % Additional condition statements
            % START
            functionFlag=true;
            % END
            r=priorFlag && functionFlag;
        end
        
        function r=invokeRuleBody(obj)
            %[TEMPLATE] invoke rule body template
            priorFlag=invokeRuleBody@AbstractRule(obj);
            % Additional invokations (yes we are summoning the rule :D)
            % START
            functionFlag=true;
            % END
            r= priorFlag && functionFlag;
        end
    end
end

