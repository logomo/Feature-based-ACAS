classdef AbstractRule<LoggableObject
    %ABSTRACTRULE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        context
        jointPointCode
        ruleCode
        ruleEngine
        utmControl
    end
    
    methods
        function obj = AbstractRule(context,jointPointCode,ruleCode)
            Cmnf.logc(obj, ['Invoking rule:   ', RuleCode.toString(ruleCode)]);
            Cmnf.logc(obj, [' -- joint point: ', RuleJointPoint.toString(jointPointCode)]);
            contextString ='';
            keys=context.keys;
            for k=1:length(context.keys)
                contextString=[contextString,keys{k},'; '];
            end
            Cmnf.logc(obj, [' -- context: ', contextString]);
            obj.context=context;
            obj.jointPointCode=jointPointCode;
            obj.ruleCode=ruleCode;
        end
        
        function r=parseContext(obj)
            Cmnf.logc(obj, ['--START-- Parse context:   ', RuleCode.toString(obj.ruleCode)]);
            obj.ruleEngine=obj.context('ruleEngine');
            if isobject(obj.ruleEngine) && strcmp(class(obj.ruleEngine),'RuleEngine')
                Cmnf.logc(obj, [' -- RuleEngine loaded to obj.ruleEngine']);
            else
                r=false;
                return;
            end
            obj.utmControl=obj.context('utmControl');
            if isobject(obj.utmControl) && strcmp(class(obj.utmControl),'UTMControl')
                Cmnf.logc(obj, [' -- UTMControl loaded to obj.utmControl']);
            else
                r=false;
                return;
            end
            r=true;
        end
        
        function r=testCondition(obj)
            Cmnf.logc(obj, ['--START-- Condition test:   ', RuleCode.toString(obj.ruleCode)]);
            r=true;
        end
        
        function r=invokeRuleBody(obj)
            Cmnf.logc(obj, ['--START-- Invoke rule body:   ', RuleCode.toString(obj.ruleCode)]);
            r=true;
        end
        
        function r=runRule(obj)
            parseFlag = obj.parseContext();
            Cmnf.logc(obj, ['-- END -- Parse context:   ', RuleCode.toString(obj.ruleCode)]);
            conditionFlag = obj.testCondition();
            Cmnf.logc(obj, ['--END-- Condition test:   ', RuleCode.toString(obj.ruleCode)]);
            if parseFlag&&conditionFlag
                Cmnf.logc(obj, ['--SUCCESS-- conndition passed']);
                obj.invokeRuleBody();
                Cmnf.logc(obj, ['--END-- Invoke rule body:   ', RuleCode.toString(obj.ruleCode)]);
            else
                Cmnf.logc(obj, ['--FAILED-- conndition failed']);
            end
        end
    end
end

