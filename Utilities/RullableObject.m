classdef RullableObject<LoggableObject
    %RullableObject [ABSTRACT] Class for object where is possible to apply rule engine
    
    properties
        ruleEngine=0;               % rule engine is not initialized
        reContext=0;                % rule engine context not initialized  
        reParams=0;                 % rule engine parameters not initialized
    end
    
    methods
        %% Rule engine getter
        function r=getRuleEngine(obj)
            % Get rule engine instance (Singleton)
            r=obj.ruleEngine;
        end
        
        %% Create context for rule engine
        function r=createContextRuleEngine(obj)
            % Create context for rule engine, as map
            obj.reContext('ruleEngine')=obj.ruleEngine;
            obj.reContext('obj')=obj;
            r=obj.reContext;
        end
        
        %% Get RuleEngine Context - instance 
        function r=getContextRuleEngine(obj)
            %Get RuleEngine Context - instance 
            r=obj.reContext;
        end
        
        %% Append context
        function r=appendContextRuleEngine(obj,master)
            %Append context with master entries
            obj.reContext=master.copyContextRuleEngine;
            r=obj.createContextRuleEngine;
        end
        
        %% Copy RuleEngine Contex - rule body
        function r=copyContextRuleEngine(obj)
            %Get the map copy of the context
            r=Cmnf.copyMap(obj.reContext);
        end
        
        
        %% Get Rule Engine Parameters - instance
        function r=getParametersRuleEngine(obj)
            %Get our instance of parameters
            r=obj.reParams;
        end
        
        %% Copy Rule Engine Parameters -ruleBody
        function r=copyParametersRuleEngine(obj)
            %Get the parameters in safe manner for rule body implementation
            r=Cmnf.copyMap(obj.reParams);
        end
        
        %% Append context
        function r=appendParametersRuleEngine(obj,master)
            %append existing context by master content
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
            %Load rule engine data structure into instance class
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

