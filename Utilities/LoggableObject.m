classdef LoggableObject<handle
    %LOGGABLEOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        traceLog;
    end
    
    methods
        function trace(obj)
            fprintf(obj.traceLog);
        end
    end
    
end

