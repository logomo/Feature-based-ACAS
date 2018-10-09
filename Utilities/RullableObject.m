classdef RullableObject<LoggableObject
    
    
    properties
        ruleEngine=0;               % rule engine is not initialized
        reContext=0;                % rule engine context not initialized  
        reParams=0;                 % rule engine parameters not initialized
    end
    
    methods
        %% Rule engine getter
        function r=getRuleEngine(obj)
            r=obj.ruleEngine;
        end
        
        %% Create context for rule engine
        function r=createContextRuleEngine(obj)
            obj.reContext('ruleEngine')=obj.ruleEngine;
            obj.reContext('obj')=obj;
            r=obj.reContext;
        end
        
        %% Get RuleEngine Context - instance 
        function r=getContextRuleEngine(obj)
            r=obj.reContext;
        end
        
        %% Append context
        function r=appendContextRuleEngine(obj,master)
            obj.reContext=master.copyContextRuleEngine;
            r=obj.createContextRuleEngine;
        end
        
        %% Copy RuleEngine Contex - rule body
        function r=copyContextRuleEngine(obj)
            r=Cmnf.copyMap(obj.reContext);
        end
        
        
        %% Get Rule Engine Parameters - instance
        function r=getParametersRuleEngine(obj)
            r=obj.reParams;
        end
        
        %% Copy Rule Engine Parameters -ruleBody
        function r=copyParametersRuleEngine(obj)
            r=Cmnf.copyMap(obj.reParams);
        end
        
        %% Append context
        function r=appendParametersRuleEngine(obj,master)
            oldMap=obj.reParams;
            keys=oldMap.keys;
            obj.reParams=master.copyParametersRuleEngine;
            for k=1:length(keys)%Override context of cloned context ...
                key=keys{k};
                value=oldMap(key);
                obj.reParams(key)=value;
            end
            r=obj.reParams;
        end
        
        %% Inject rule engine 
        % Default method (target for override)
        function r=injectRuleEngine(obj,ruleEngine)
            if obj.ruleEngine==0
                obj.ruleEngine=ruleEngine;
                obj.reContext=containers.Map;
                obj.createContextRuleEngine;
                obj.reParams=containers.Map;
                r=true;
            else
                r=false;
            end
        end
    end
end

