classdef LoggableObject<handle
    %LOGGABLEOBJECT [Abstract] Object keeping own functionality log
    
    
    properties
        traceLog; %string containing logged data
    end
    
    methods
        function trace(obj)
            %Print the containment of tracelog
            fprintf(obj.traceLog); 
        end
    end
    
end

